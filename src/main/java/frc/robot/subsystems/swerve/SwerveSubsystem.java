package frc.robot.subsystems.swerve;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightEstimator;
import frc.robot.subsystems.vision.VisionEstimateResult;
import frc.robot.util.PosePublisher;
import frc.robot.util.Util;

import static frc.robot.Config.Swerve.*;
import static frc.robot.Config.Odometry.*;

/**
 * Swerve drive subsystem with odometry and pose estimation.
 * <p>
 * Adapted from Robohawk2022/2025CompetitionRobot, converted from
 * REV SparkMax to CTRE TalonFX.
 */
public class SwerveSubsystem extends SubsystemBase {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

//region Implementation --------------------------------------------------------

    private final SwerveHardware hardware;

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;

    private String currentMode = "idle";

    private double maxSpeedMps;
    private double maxRotationRps;

    private VisionEstimateResult latestVisionResult;

    // cached pose values (hawk-lib pattern)
    private Pose2d latestOdometryPose = new Pose2d();
    private Pose2d latestFusedPose = new Pose2d();

    // reset tracking for dashboard
    private int resetCount = 0;

    // periodic logging counter (logs every 50 cycles = 1 second)
    private int logCounter = 0;

    // high-frequency odometry (may be null if not supported or disabled)
    private final OdometryThread odometryThread;
    private final OdometryDiagnostics diagnostics;

    ChassisSpeeds latistspeed = Util.ZERO_SPEED;

    /**
     * Creates a {@link SwerveSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public SwerveSubsystem(SwerveHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);

        // initialize odometry at origin
        odometry = new SwerveDriveOdometry(
                KINEMATICS,
                hardware.getHeading(),
                hardware.getModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                hardware.getHeading(),
                hardware.getModulePositions(),
                new Pose2d());

        // Field2d for AdvantageScope visualization
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);

        // set initial max speeds from config
        updateSpeedLimits();

        // initialize diagnostics
        diagnostics = new OdometryDiagnostics();

        // initialize high-frequency odometry if supported and enabled
        if (hardware.supportsHighFrequencyOdometry() && enableHF.getAsBoolean()) {
            odometryThread = new OdometryThread(hardware, HF_FREQUENCY, POSE_HISTORY_SIZE);
            // the high-frequency odometry thread overwhelms our CAN bus
            // TODO - fix me
            // odometryThread.start();
            Util.log("High-frequency odometry enabled at %.0f Hz", HF_FREQUENCY);
        } else {
            odometryThread = null;
            if (!hardware.supportsHighFrequencyOdometry()) {
                Util.log("High-frequency odometry not supported by hardware");
            } else {
                Util.log("High-frequency odometry disabled by config");
            }
        }

        // dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("HeadingDeg", () -> getHeading().getDegrees(), null);
            builder.addDoubleProperty("PoseXFeet", () -> Units.metersToFeet(getPose().getX()), null);
            builder.addDoubleProperty("PoseYFeet", () -> Units.metersToFeet(getPose().getY()), null);
            builder.addDoubleProperty("Speedx", () -> latistspeed.vxMetersPerSecond, null);
            builder.addDoubleProperty("Speedy", () -> latistspeed.vyMetersPerSecond, null);
            builder.addDoubleProperty("SpeedO", () -> latistspeed.omegaRadiansPerSecond, null);

            // reset odometry button - click to reset pose and heading to origin
            builder.addBooleanProperty("ResetOdometry", () -> false, (value) -> {
                if (value) {
                    zeroHeading();
                    resetPose(new Pose2d());
                    Util.log("Odometry and heading reset from dashboard");
                }
            });

            // reset counter to verify resets are happening
            builder.addIntegerProperty("ResetCount", () -> resetCount, null);

            if (verboseLogging) {
                builder.addDoubleProperty("MaxSpeedFPS", () -> Units.metersToFeet(maxSpeedMps), null);
                builder.addDoubleProperty("MaxRotationDPS", () -> Math.toDegrees(maxRotationRps), null);
            }
        });
    }

    /**
     * Updates speed limits from preferences. Call periodically to allow
     * runtime tuning.
     */
    private void updateSpeedLimits() {
        maxSpeedMps = Units.feetToMeters(maxSpeedFps.getAsDouble());
        maxRotationRps = Math.toRadians(maxRotationDps.getAsDouble());
    }

    @Override
    public void periodic() {
        // update odometry - use HF thread if available, otherwise standard 50Hz
        Rotation2d heading = hardware.getHeading();
        SwerveModulePosition[] positions = hardware.getModulePositions();

        if (odometryThread != null) {
            // HF thread handles odometry at 250Hz
            // Update the standard odometry too for comparison/fallback
            odometry.update(heading, positions);
            latestOdometryPose = odometryThread.getLatestPose();
        } else {
            // standard 50Hz odometry update
            odometry.update(heading, positions);
            latestOdometryPose = odometry.getPoseMeters();
        }

        // always update the pose estimator (vision integration happens here)
        poseEstimator.update(heading, positions);
        latestFusedPose = poseEstimator.getEstimatedPosition();

        // update vision pose estimation
        latestVisionResult = LimelightEstimator.updateEstimate(
                poseEstimator,
                heading.getDegrees(),
                hardware.getYawRateDps(),
                poseEstimator.getEstimatedPosition());

        // update diagnostics
        if (enableDiagnostics.getAsBoolean()) {
            Pose2d odomPose = odometryThread != null
                    ? odometryThread.getLatestPose()
                    : odometry.getPoseMeters();
            diagnostics.update(odomPose, poseEstimator.getEstimatedPosition(),
                    latestVisionResult, odometryThread);
            diagnostics.publishTelemetry();

            // warn if drift exceeds threshold
            double driftThreshold = driftWarning.getAsDouble();
            if (diagnostics.getCurrentDrift() > driftThreshold) {
                Util.log("WARNING: Odometry drift %.3fm exceeds threshold %.3fm",
                        diagnostics.getCurrentDrift(), driftThreshold);
            }
        }

        // update Field2d for AdvantageScope
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());

        // publish poses for AdvantageScope
        Pose2d odomPose = odometryThread != null
                ? odometryThread.getLatestPose()
                : odometry.getPoseMeters();
        PosePublisher.publish("Odometry", odomPose);
        PosePublisher.publish("Estimated", poseEstimator.getEstimatedPosition());

        // publish vision pose if valid
        if (latestVisionResult != null && latestVisionResult.accepted && latestVisionResult.getPose() != null) {
            PosePublisher.publish("Vision", latestVisionResult.getPose());
        }
    }

    /**
     * @return the current robot heading from the gyro
     */
    public Rotation2d getHeading() {
        return hardware.getHeading();
    }

    /**
     * @return the current estimated pose of the robot (fused with vision)
     */
    public Pose2d getPose() {
        return latestFusedPose;
    }

    /**
     * @return the odometry-only pose (no vision correction)
     */
    public Pose2d getOdometryPose() {
        return latestOdometryPose;
    }

    /**
     * Resets the odometry to a specific pose.
     * <p>
     * Uses the current gyro heading and module positions (hawk-lib pattern).
     *
     * @param newPose the pose to reset to
     */
    public void resetPose(Pose2d newPose) {
        Rotation2d currentHeading = hardware.getHeading();
        SwerveModulePosition[] currentPositions = hardware.getModulePositions();

        // reset odometry
        odometry.resetPosition(currentHeading, currentPositions, newPose);
        latestOdometryPose = newPose;

        // reset pose estimator
        poseEstimator.resetPosition(currentHeading, currentPositions, newPose);
        latestFusedPose = newPose;

        // reset high-frequency odometry thread if present
        if (odometryThread != null) {
            odometryThread.resetPose(newPose);
        }

        // reset vision baseline for jump detection
        LimelightEstimator.resetLastPose();

        // reset diagnostics
        diagnostics.reset();

        // increment counter for dashboard
        resetCount++;
    }

    /**
     * Zeroes the gyro heading.
     */
    public void zeroHeading() {
        hardware.zeroHeading();
    }

    /**
     * Drives the robot using the given speeds.
     *
     * @param speeds        chassis speeds (vx, vy in m/s, omega in rad/s)
     * @param fieldRelative whether speeds are field-relative
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
        }

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMps);
        hardware.setModuleStates(states);
        latistspeed = speeds;
    }

    /**
     * @return the current chassis speeds
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return KINEMATICS.toChassisSpeeds(hardware.getModuleStates());
    }

    /**
     * Sets the wheels to an X pattern to prevent movement.
     */
    public void setX() {
        hardware.setX();
    }

    /**
     * @return whether the latest vision estimate was accepted
     */
    public boolean isVisionValid() {
        return latestVisionResult != null && latestVisionResult.accepted;
    }

    /**
     * @return the latest vision estimate result (may be null or rejected)
     */
    public VisionEstimateResult getLatestVisionResult() {
        return latestVisionResult;
    }

    /**
     * @return the odometry diagnostics tracker
     */
    public OdometryDiagnostics getDiagnostics() {
        return diagnostics;
    }

    /**
     * @return the high-frequency odometry thread (may be null if not supported)
     */
    public OdometryThread getOdometryThread() {
        return odometryThread;
    }

    /**
     * @return true if high-frequency odometry is running
     */
    public boolean isHighFrequencyOdometryEnabled() {
        return odometryThread != null && odometryThread.isRunning();
    }

    /**
     * Gets the odometry pose at a specific timestamp for latency compensation.
     *
     * @param timestamp the FPGA timestamp in seconds
     * @return the interpolated pose at that time, or current pose if HF not available
     */
    public Pose2d getOdometryPoseAtTime(double timestamp) {
        if (odometryThread != null) {
            return odometryThread.getPoseAtTime(timestamp);
        }
        return odometry.getPoseMeters();
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that holds position (sets X pattern)
     */
    public Command idleCommand() {
        return run(() -> {
            currentMode = "idle";
            setX();
        });
    }

    /**
     * Creates a teleop drive command.
     *
     * @param xSupplier     forward/backward speed (-1 to 1)
     * @param ySupplier     left/right speed (-1 to 1)
     * @param rotSupplier   rotation speed (-1 to 1)
     * @param fieldRelative whether to drive field-relative
     * @return the drive command
     */
    public Command driveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier,
            boolean fieldRelative) {

        return run(() -> {
            currentMode = fieldRelative ? "teleop-field" : "teleop-robot";
            updateSpeedLimits();

            double vx = xSupplier.getAsDouble() * maxSpeedMps;
            double vy = ySupplier.getAsDouble() * maxSpeedMps;
            double omega = rotSupplier.getAsDouble() * maxRotationRps;

            drive(new ChassisSpeeds(vx, vy, omega), fieldRelative);
        });
    }

    /**
     * @return a command that zeroes the gyro heading
     */
    public Command zeroHeadingCommand() {
        return runOnce(() -> {
            zeroHeading();
            Util.log("Gyro zeroed");
        });
    }

    /**
     * @return a command that zeros the heading but keeps the current X/Y position
     */
    public Command resetPoseCommand() {
        return runOnce(() -> {
            // keep current X/Y, just zero the heading
            Pose2d current = getPose();
            Pose2d newPose = new Pose2d(current.getX(), current.getY(), new Rotation2d());

            zeroHeading();
            resetPose(newPose);

            Util.log("Heading reset - X:%.2f Y:%.2f Heading:%.1f",
                    getPose().getX(), getPose().getY(), getHeading().getDegrees());
        });
    }

    /**
     * Creates a command that drives at fixed speeds.
     *
     * @param speeds  the speeds to drive at
     * @param seconds how long to drive
     * @return the command
     */
    public Command fixedSpeedCommand(ChassisSpeeds speeds, double seconds) {
        return run(() -> {
            currentMode = "fixed";
            drive(speeds, false);
        }).withTimeout(seconds);
    }

//endregion
}

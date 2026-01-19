package frc.robot.subsystems.swerve;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightEstimator;
import frc.robot.subsystems.vision.VisionEstimateResult;
import frc.robot.util.PosePublisher;
import frc.robot.util.Util;

import static frc.robot.Config.Swerve.*;
import static frc.robot.Config.Odometry.enableDiagnostics;
import static frc.robot.Config.Odometry.driftWarning;

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
    private boolean turboActive = false;
    private boolean sniperActive = false;

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

        // dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addBooleanProperty("Enabled?", DriverStation::isEnabled, null);
            builder.addBooleanProperty("TurboActive?", () -> turboActive, null);
            builder.addBooleanProperty("SniperActive?", () -> sniperActive, null);
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
        // refresh all hardware signals in a single CAN transaction (reduces CAN bus traffic)
        hardware.refreshSignals();

        // update odometry (standard 50Hz)
        Rotation2d heading = hardware.getHeading();
        SwerveModulePosition[] positions = hardware.getModulePositions();

        odometry.update(heading, positions);
        latestOdometryPose = odometry.getPoseMeters();

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
            diagnostics.update(latestOdometryPose, poseEstimator.getEstimatedPosition(),
                    latestVisionResult);
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
        PosePublisher.publish("Odometry", latestOdometryPose);
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
        drive(speeds, fieldRelative, maxSpeedMps);
    }

    /**
     * Drives the robot using the given speeds with a custom max speed limit.
     *
     * @param speeds        chassis speeds (vx, vy in m/s, omega in rad/s)
     * @param fieldRelative whether speeds are field-relative
     * @param maxSpeed      maximum wheel speed for desaturation (m/s)
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double maxSpeed) {
        if (fieldRelative) {
            speeds = Util.fromDriverRelativeSpeeds(speeds, getHeading());
        }

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
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
     * Creates a teleop drive command with turbo and sniper speed modes.
     * <p>
     * Speed modes are activated by triggers:
     * <ul>
     *   <li>Sniper (left trigger > 0.5): Slow, precise movement</li>
     *   <li>Turbo (right trigger > 0.5): Fast movement</li>
     *   <li>Sniper takes priority if both triggers are pressed</li>
     * </ul>
     *
     * @param xSupplier      forward/backward speed (-1 to 1)
     * @param ySupplier      left/right speed (-1 to 1)
     * @param rotSupplier    rotation speed (-1 to 1)
     * @param sniperTrigger  sniper mode trigger axis (0 to 1)
     * @param turboTrigger   turbo mode trigger axis (0 to 1)
     * @param fieldRelative  whether to drive field-relative
     * @return the drive command
     */
    public Command driveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier,
            DoubleSupplier sniperTrigger,
            DoubleSupplier turboTrigger,
            boolean fieldRelative) {

        return run(() -> {
            updateSpeedLimits();

            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            double rot = rotSupplier.getAsDouble();

            // check trigger thresholds
            sniperActive = sniperTrigger.getAsDouble() > 0.5;
            turboActive = turboTrigger.getAsDouble() > 0.5;

            // clamp inputs to [-1, 1]
            x = MathUtil.clamp(x, -1.0, 1.0);
            y = MathUtil.clamp(y, -1.0, 1.0);
            rot = MathUtil.clamp(rot, -1.0, 1.0);

            // convert to velocities
            double vx = x * maxSpeedMps;
            double vy = y * maxSpeedMps;
            double omega = rot * maxRotationRps;

            // track effective max speed for desaturation
            double effectiveMaxSpeed = maxSpeedMps;

            // apply speed modifiers (sniper takes priority)
            if (sniperActive) {
                double sf = sniperFactor.getAsDouble();
                vx *= sf;
                vy *= sf;
                effectiveMaxSpeed *= sf;
                if (applySniperToRotation.getAsBoolean()) {
                    omega *= sf;
                }
                currentMode = fieldRelative ? "teleop-sniper" : "teleop-sniper-robot";
            } else if (turboActive) {
                double tf = turboFactor.getAsDouble();
                vx *= tf;
                vy *= tf;
                effectiveMaxSpeed *= tf;
                currentMode = fieldRelative ? "teleop-turbo" : "teleop-turbo-robot";
            } else {
                currentMode = fieldRelative ? "teleop-field" : "teleop-robot";
            }

            drive(new ChassisSpeeds(vx, vy, omega), fieldRelative, effectiveMaxSpeed);
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

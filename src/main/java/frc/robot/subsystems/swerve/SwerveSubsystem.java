package frc.robot.subsystems.swerve;

import java.util.Objects;

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
import frc.robot.GameController;
import frc.robot.commands.swerve.SwerveOrbitCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.subsystems.vision.LimelightEstimator;
import frc.robot.subsystems.vision.VisionEstimateResult;
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

//region Members & constructor -------------------------------------------------

    private final SwerveHardware hardware;

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;

    private String currentMode = "idle";

    private VisionEstimateResult latestVisionResult;

    // cached pose values (hawk-lib pattern)
    private Pose2d latestOdometryPose = new Pose2d();
    private Pose2d latestFusedPose = new Pose2d();

    // reset tracking for dashboard
    private int resetCount = 0;

    private final OdometryDiagnostics diagnostics;

    ChassisSpeeds latestSpeed = Util.ZERO_SPEED;

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

        // initialize diagnostics
        diagnostics = new OdometryDiagnostics();

        // dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addBooleanProperty("Enabled?", DriverStation::isEnabled, null);
            if (verboseLogging) {
                builder.addDoubleProperty("PoseX", () -> Units.metersToFeet(getPose().getX()), null);
                builder.addDoubleProperty("PoseY", () -> Units.metersToFeet(getPose().getY()), null);
                builder.addDoubleProperty("PoseOmega", () -> getHeading().getDegrees(), null);
                builder.addDoubleProperty("SpeedX", () -> Units.metersToFeet(latestSpeed.vxMetersPerSecond), null);
                builder.addDoubleProperty("SpeedY", () -> Units.metersToFeet(latestSpeed.vyMetersPerSecond), null);
                builder.addDoubleProperty("SpeedOmega", () -> Units.radiansToDegrees(latestSpeed.omegaRadiansPerSecond), null);
            }

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
        });
    }

//endregion

//region Periodic --------------------------------------------------------------

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
        Util.publishPose("Odometry", latestOdometryPose);
        Util.publishPose("Estimated", poseEstimator.getEstimatedPosition());

        // publish vision pose if valid
        if (latestVisionResult != null && latestVisionResult.accepted && latestVisionResult.getPose() != null) {
            Util.publishPose("Vision", latestVisionResult.getPose());
        }
    }

//endregion

//region Pose accessors --------------------------------------------------------

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

//endregion

//region Driving ---------------------------------------------------------------

    /**
     * @return the current chassis speeds
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return KINEMATICS.toChassisSpeeds(hardware.getModuleStates());
    }

    /**
     * Drives the robot using robot-relative speeds.
     * @param mode the mode to display for debugging
     * @param speeds robot-relative chassis speeds (vx, vy in m/s, omega in rad/s)
     */
    public void driveRobotRelative(String mode, ChassisSpeeds speeds) {
        currentMode = mode;

        // Compensate for translational skew during rotation (20ms timestep)
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, Util.DT);

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(discretizedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED_MPS);
        hardware.setModuleStates(states);
        latestSpeed = discretizedSpeeds;
    }

    /**
     * Drives using pre-computed module states.
     * <p>
     * Used by commands that need custom desaturation logic (e.g., rotation-priority).
     *
     * @param mode the mode to display for debugging
     * @param states the module states to apply
     * @param speeds the chassis speeds for telemetry
     */
    public void driveStates(String mode, SwerveModuleState [] states, ChassisSpeeds speeds) {
        currentMode = mode;
        hardware.setModuleStates(states);
        latestSpeed = speeds;
    }

//endregion

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
     * @return a command that resets all wheel angles to 0 (forward facing)
     */
    public Command resetWheelsCommand() {
        return runOnce(() -> {
            hardware.lockTurnMotors();
            Util.log("Wheels reset to forward");
        });
    }

    /**
     * Creates a teleop drive command from controller inputs.
     *
     * @param controller the game controller for driver input
     * @return the drive command (field-relative)
     * @see SwerveTeleopCommand
     */
    public Command driveCommand(GameController controller) {
        return new SwerveTeleopCommand(this, controller);
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
        return run(() -> driveRobotRelative("auto", speeds))
                .withTimeout(seconds);
    }

    /**
     * Creates an orbit drive command that orbits around the target (Tower) while facing it.
     *
     * @param controller the game controller for driver input
     * @return the orbit command
     * @see SwerveOrbitCommand
     */
    public Command orbitCommand(GameController controller) {
        return new SwerveOrbitCommand(this, controller);
    }

//endregion
}

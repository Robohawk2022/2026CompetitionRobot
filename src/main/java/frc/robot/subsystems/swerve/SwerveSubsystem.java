package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameController;
import frc.robot.commands.swerve.SwerveOrbitCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.swerve.SwerveToHeadingCommand;
import frc.robot.commands.swerve.SwerveToPoseCommand;
import frc.robot.util.Field;
import frc.robot.util.Util;

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

    final SwerveHardware hardware;
    final SwerveDriveOdometry odometry;
    final SwerveDrivePoseEstimator poseEstimator;
    final List<Consumer<Pose2d>> poseResetListeners;
    final Field2d field2d;

    String currentMode = "idle";
    Pose2d latestOdometryPose;
    Pose2d latestFusedPose;
    int resetCount = 0;
    ChassisSpeeds latestSpeed;
    boolean wheelsLocked;
    long visionPoseCount;

    /**
     * Creates a {@link SwerveSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public SwerveSubsystem(SwerveHardware hardware) {

        this.hardware = Objects.requireNonNull(hardware);

        // initialize odometry and pose estimates
        this.odometry = new SwerveDriveOdometry(
                SwerveHardwareConfig.KINEMATICS,
                hardware.getHeading(),
                hardware.getModulePositions(),
                Pose2d.kZero);
        this.poseEstimator = new SwerveDrivePoseEstimator(
                SwerveHardwareConfig.KINEMATICS,
                hardware.getHeading(),
                hardware.getModulePositions(),
                Pose2d.kZero);
        this.poseResetListeners = new ArrayList<>();
        this.latestOdometryPose = Pose2d.kZero;
        this.latestFusedPose = Pose2d.kZero;
        this.latestSpeed = Util.ZERO_SPEED;
        this.wheelsLocked = false;

        // initialize Field2d for AdvantageScope visualization
        this.field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);

        // zero the gyro heading and go through the motions of pose reset
        resetPose(Pose2d.kZero);

        // dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {

            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addBooleanProperty("WheelsLocked?", () -> wheelsLocked, null);
            builder.addIntegerProperty("ResetCount", () -> resetCount, null);
            builder.addBooleanProperty("ResetOdometry", () -> false, (value) -> {
                if (value) {
                    Util.log("[swerve] odometry reset requests from dashboard");
                    resetPose(Pose2d.kZero);
                }
            });

            if (verboseLogging) {
                builder.addDoubleProperty("PoseX", () -> Units.metersToFeet(getPose().getX()), null);
                builder.addDoubleProperty("PoseY", () -> Units.metersToFeet(getPose().getY()), null);
                builder.addDoubleProperty("PoseOmega", () -> getHeading().getDegrees(), null);
                builder.addDoubleProperty("SpeedX", () -> Units.metersToFeet(latestSpeed.vxMetersPerSecond), null);
                builder.addDoubleProperty("SpeedY", () -> Units.metersToFeet(latestSpeed.vyMetersPerSecond), null);
                builder.addDoubleProperty("SpeedOmega", () -> Units.radiansToDegrees(latestSpeed.omegaRadiansPerSecond), null);
                builder.addIntegerProperty("VisionPoseCount", () -> visionPoseCount, null);
            }
        });
    }

//endregion

//region Periodic --------------------------------------------------------------

    @Override
    public void periodic() {

        // refresh all hardware signals in a single CAN transaction
        hardware.refreshSignals();

        // update odometry and pose estimator
        Rotation2d heading = hardware.getHeading();
        SwerveModulePosition[] positions = hardware.getModulePositions();
        odometry.update(heading, positions);
        poseEstimator.update(heading, positions);

        // cache pose estimates
        latestOdometryPose = odometry.getPoseMeters();
        latestFusedPose = poseEstimator.getEstimatedPosition();

        // publish pose information
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
        Util.publishPose("Odometry", latestOdometryPose);
        Util.publishPose("Fused", poseEstimator.getEstimatedPosition());
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
     * @return the current estimated pose (odometry fused with vision)
     */
    public Pose2d getPose() {
        return latestFusedPose;
    }

    /**
     * @return the current odometry-only pose estimate
     */
    public Pose2d getOdometryPose() {
        return latestOdometryPose;
    }

    /**
     * Submits a vision pose estimate
     * @param estimatedPose the estimated pose
     * @param timestamp the age of the estimate in seconds
     * @param confidence confidence in the estimate
     */
    public void submitVisionEstimate(Pose2d estimatedPose, double timestamp, Vector<N3> confidence) {
        if (estimatedPose != null) {
            poseEstimator.addVisionMeasurement(estimatedPose, timestamp, confidence);
            visionPoseCount++;
        }
    }

    /**
     * Zeros the gyro and resets the odometry to a specific pose
     * @param newPose the pose to reset to
     */
    public void resetPose(Pose2d newPose) {

        hardware.zeroHeading();;

        Rotation2d currentHeading = hardware.getHeading();
        SwerveModulePosition[] currentPositions = hardware.getModulePositions();

        // reset odometry and fused pose estimator
        odometry.resetPosition(currentHeading, currentPositions, newPose);
        poseEstimator.resetPosition(currentHeading, currentPositions, newPose);

        // reset pose estimates & update listeners
        latestOdometryPose = newPose;
        latestFusedPose = newPose;
        for (Consumer<Pose2d> listener : poseResetListeners) {
            listener.accept(newPose);
        }

        // increment counter for dashboard
        resetCount++;

        Util.log("[swerve] reset pose to %s", newPose);
    }

//endregion

//region Driving ---------------------------------------------------------------

    /**
     * @return the current chassis speeds
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return SwerveHardwareConfig.KINEMATICS.toChassisSpeeds(hardware.getModuleStates());
    }

    /**
     * Drives the robot using robot-relative speeds
     * @param mode the mode to display for debugging
     * @param speeds robot-relative chassis speeds (vx, vy in m/s, omega in rad/s)
     */
    public void driveRobotRelative(String mode, ChassisSpeeds speeds) {
        driveRobotRelative(mode, speeds, null);
    }

    /**
     * Drives the robot using robot-relative speeds and a custom center of
     * rotation
     *
     * @param mode the mode to display for debugging
     * @param speeds robot-relative chassis speeds (vx, vy in m/s, omega in rad/s)
     * @param center the center of rotation (null means robot center)
     */
    public void driveRobotRelative(String mode, ChassisSpeeds speeds, Translation2d center) {
        currentMode = mode;

        if (center == null) {
            center = Translation2d.kZero;
        }

        // Compensate for translational skew during rotation (20ms timestep)
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, Util.DT);

        SwerveModuleState[] states = SwerveHardwareConfig.KINEMATICS.toSwerveModuleStates(discretizedSpeeds, center);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveHardwareConfig.MAX_WHEEL_SPEED_MPS);
        hardware.setModuleStates(states);
        latestSpeed = discretizedSpeeds;
    }

//endregion

    /**
     * Sets the wheels to an X pattern to prevent movement.
     */
    public void setX() {
        hardware.setX();
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that holds position
     */
    public Command idleCommand() {
        return run(() -> driveRobotRelative("idle", Util.ZERO_SPEED));
    }

    /**
     * @return a command that locks the wheels in an X pattern
     */
    public Command lockWheelsCommand() {
        return startRun(
                () -> {
                    currentMode = "lock";
                    Util.log("[swerve] locking wheels");
                },
                hardware::setX);
    }

    /**
     * @return a command that resets all wheel angles to 0 (forward facing)
     */
    public Command resetWheelsCommand() {
        SwerveModuleState [] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState();
        states[1] = new SwerveModuleState();
        states[2] = new SwerveModuleState();
        states[3] = new SwerveModuleState();
        return runOnce(() -> {
            hardware.setModuleStates(states);
        }).finallyDo(() -> wheelsLocked = false);
    }

    /**
     * Creates a teleop drive command from controller inputs
     * @param controller the game controller for driver input
     * @return the drive command (field-relative)
     * @see SwerveTeleopCommand
     */
    public Command driveCommand(GameController controller) {
        return new SwerveTeleopCommand(this, controller);
    }

    /**
     * @return a command that zeros the heading of the robot but keeps the
     * current (X,Y) position
     */
    public Command zeroHeadingCommand() {
        return resetPoseCommand(oldPose -> new Pose2d(
                oldPose.getX(),
                oldPose.getY(),
                Rotation2d.kZero));
    }

    /**
     * @return a command that zeros the pose of the robot
     */
    public Command zeroPoseCommand() {
        return resetPoseCommand(oldPose -> Pose2d.kZero);
    }

    /**
     *
     * @return a command that zeros the heading but keeps the current X/Y position
     */
    public Command resetPoseCommand(Function<Pose2d,Pose2d> poseFunction) {
        return runOnce(() -> {
            Pose2d oldPose = getPose();
            Pose2d newPose = poseFunction.apply(oldPose);
            resetPose(newPose);
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
     * Creates an orbit drive command that orbits around the hub.
     * <p>
     * This is a compound command that:
     * <ol>
     *   <li>Determines the location of the alliance hub</li>
     *   <li>Rotates to face the hub using {@link SwerveToHeadingCommand}</li>
     *   <li>Orbits the hub using {@link SwerveOrbitCommand}</li>
     * </ol>
     *
     * @param controller the game controller for driver input
     * @return the orbit command sequence
     * @see SwerveOrbitCommand
     */
    public Command orbitCommand(GameController controller) {
        return defer(() -> {
            
            // determine the location of the hub
            Pose2d hubCenter = Field.getHubCenter();

            // calculate heading to face the hub from current position
            Rotation2d headingToHub = hubCenter.getTranslation()
                    .minus(getPose().getTranslation())
                    .getAngle();

            Util.log("[swerve] orbit: hub at %s, heading %.1f deg",
                    hubCenter, headingToHub.getDegrees());

            // sequence: rotate to face hub, then orbit
            return Commands.sequence(
                    new SwerveToHeadingCommand(this, headingToHub),
                    new SwerveOrbitCommand(this, controller, hubCenter)
            );
        });
    }

    /**
     * Creates a command that rotates to face the specified heading
     * @param heading the target heading
     * @return the heading command
     * @see SwerveToHeadingCommand
     */
    public Command driveToHeadingCommand(Rotation2d heading) {
        return new SwerveToHeadingCommand(this, heading);
    }

    /**
     * Creates a command that rotates to face a heading dynamically
     * re-calculated whenever the command executes
     * @param headingFunction function to compute the target heading
     * @return the heading command
     * @see SwerveToHeadingCommand
     */
    public Command driveToHeadingCommand(Function<Pose2d,Rotation2d> headingFunction) {
        return defer(() -> {
            Pose2d currentPose = getPose();
            Rotation2d newHeading = headingFunction.apply(currentPose);
            return new SwerveToHeadingCommand(this, newHeading);
        });
    }

    /**
     * Creates a command that drives to the specified pose along a straight line
     * @param pose the target pose
     * @return the pose command
     * @see SwerveToPoseCommand
     */
    public Command driveToPoseCommand(Pose2d pose) {
        return new SwerveToPoseCommand(this, pose);
    }

    /**
     * Creates a command that drives to a pose dynamically re-calculated
     * whenever the command executes
     * @param poseFunction function to compute the target pose
     * @return the pose command
     * @see SwerveToPoseCommand
     */
    public Command driveToPoseCommand(Function<Pose2d,Pose2d> poseFunction) {
        return defer(() -> {
            Pose2d oldPose = getPose();
            Pose2d newPose = poseFunction.apply(oldPose);
            return new SwerveToPoseCommand(this, newPose);
        });
    }

//endregion
}

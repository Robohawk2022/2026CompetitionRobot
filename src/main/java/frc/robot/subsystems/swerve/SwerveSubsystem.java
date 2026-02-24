package frc.robot.subsystems.swerve;

import java.util.Objects;
import java.util.function.Function;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameController;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.swerve.SwerveToHeadingCommand;
import frc.robot.commands.swerve.SwerveToPoseCommand;
import frc.robot.util.Util;

/**
 * Swerve drive subsystem wrapping CTRE's CommandSwerveDrivetrain.
 * <p>
 * This class provides a familiar API while leveraging CTRE's optimized
 * odometry thread and SwerveRequest-based control.
 */
public class SwerveSubsystem extends SubsystemBase {

    /** Start pose */
    static final Pose2d START_POSE = Pose2d.kZero;

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

//region Members & constructor -------------------------------------------------

    final CommandSwerveDrivetrain drivetrain;

    // Reusable SwerveRequest objects
    final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    final SwerveDeadReckoner deadReckoner;

    String currentMode = "idle";
    int resetCount = 0;
    ChassisSpeeds latestSpeed = Util.ZERO_SPEED;
    boolean wheelsLocked;
    long visionPoseCount;

    /**
     * Creates a {@link SwerveSubsystem} wrapping the CTRE drivetrain.
     *
     * @param drivetrain the CTRE CommandSwerveDrivetrain (required)
     */
    public SwerveSubsystem(CommandSwerveDrivetrain drivetrain) {

        this.drivetrain = Objects.requireNonNull(drivetrain);
        this.wheelsLocked = false;

        deadReckoner = new SwerveDeadReckoner(START_POSE);
        drivetrain.resetPose(START_POSE);

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

        // CTRE drivetrain handles odometry in its own thread
        // We just need to publish visualization data
        Pose2d pose = getPose();
        Util.publishPose("FusedPose", pose);

        // publish position prominently for Shuffleboard
        SmartDashboard.putNumber("Pose X (ft)", Units.metersToFeet(pose.getX()));
        SmartDashboard.putNumber("Pose Y (ft)", Units.metersToFeet(pose.getY()));
        SmartDashboard.putNumber("Pose Heading (deg)", pose.getRotation().getDegrees());

        // publish pose for PathPlanner telemetry (meters, radians)
        SmartDashboard.putNumberArray("PathPlanner/OdometryPose",
            new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
    }

    @Override
    public void simulationPeriodic() {

        // in simulation we will update our dead reckoning pose
        deadReckoner.update(latestSpeed);

        // we will also publish the (incorrect) CTRE pose so we can laugh at it
        Util.publishPose("CtrePose", drivetrain.getState().Pose);
    }

//endregion

//region Pose accessors --------------------------------------------------------

    /**
     * @return the current robot heading from the gyro
     */
    public Rotation2d getHeading() {

        // if we are in simulation, we will calculate our pose via dead reckoning
        // TODO WTF is wrong with the CTRE simulator?
        return RobotBase.isSimulation()
                ? getPose().getRotation()
                : drivetrain.getState().Pose.getRotation();
    }

    /**
     * @return the current estimated pose (odometry fused with vision)
     */
    public Pose2d getPose() {

        // if we are in simulation, we will calculate our pose via dead reckoning
        // TODO WTF is wrong with the CTRE simulator?
       return RobotBase.isSimulation()
                ? deadReckoner.getPose()
                : drivetrain.getState().Pose;
    }

    /**
     * @return the current chassis speeds from the drivetrain
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return drivetrain.getState().Speeds;
    }

    /**
     * Submits a vision pose estimate to the CTRE pose estimator.
     *
     * @param estimatedPose the estimated pose
     * @param timestamp the timestamp of the estimate in seconds
     * @param confidence standard deviations [x, y, theta]
     */
    public void submitVisionEstimate(Pose2d estimatedPose, double timestamp, Matrix<N3, N1> confidence) {
        if (estimatedPose != null) {
            drivetrain.addVisionMeasurement(estimatedPose, timestamp, confidence);
            visionPoseCount++;
        }
    }

    /**
     * Zeros the gyro and resets the odometry to a specific pose.
     *
     * @param newPose the pose to reset to
     */
    public void resetPose(Pose2d newPose) {
        drivetrain.resetPose(newPose);
        deadReckoner.resetPose(newPose);
        resetCount++;
        Util.log("[swerve] reset pose to %s", newPose);
    }

//endregion

//region Driving ---------------------------------------------------------------

    /**
     * Drives the robot using robot-relative speeds.
     *
     * @param mode the mode to display for debugging
     * @param speeds robot-relative chassis speeds (vx, vy in m/s, omega in rad/s)
     */
    public void driveRobotRelative(String mode, ChassisSpeeds speeds) {
        driveRobotRelative(mode, speeds, null);
    }

    /**
     * Drives the robot using robot-relative speeds and a custom center of rotation.
     *
     * @param mode the mode to display for debugging
     * @param speeds robot-relative chassis speeds (vx, vy in m/s, omega in rad/s)
     * @param center the center of rotation (null means robot center)
     */
    public void driveRobotRelative(String mode, ChassisSpeeds speeds, Translation2d center) {

        currentMode = mode;
        latestSpeed = speeds;

        if (center == null) {
            center = Translation2d.kZero;
        }

        drivetrain.setControl(
            robotCentricRequest
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
                .withCenterOfRotation(center));
    }

    /**
     * Sets the wheels to an X pattern to prevent movement.
     */
    public void setX() {
        currentMode = "lock";
        drivetrain.setControl(brakeRequest);
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
                    Util.log("[swerve] locking wheels");
                },
                this::setX);
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
     * @param poseFunction function to compute new pose from old pose
     * @return a command that resets the pose using the function
     */
    public Command resetPoseCommand(Function<Pose2d,Pose2d> poseFunction) {
        return runOnce(() -> {
            Pose2d oldPose = getPose();
            Pose2d newPose = poseFunction.apply(oldPose);
            resetPose(newPose);
        });
    }

    /**
     * Creates a command that rotates to face the specified heading.
     *
     * @param heading the target heading
     * @return the heading command
     * @see SwerveToHeadingCommand
     */
    public Command driveToHeadingCommand(Rotation2d heading) {
        return new SwerveToHeadingCommand(this, heading);
    }

    /**
     * Creates a command that drives to the specified pose along a straight line.
     *
     * @param pose the target pose
     * @return the pose command
     * @see SwerveToPoseCommand
     */
    public Command driveToPoseCommand(Pose2d pose) {
        return new SwerveToPoseCommand(this, pose);
    }

//endregion
}

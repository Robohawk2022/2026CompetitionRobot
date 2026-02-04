package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
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
 * Swerve drive subsystem wrapping CTRE's CommandSwerveDrivetrain.
 * <p>
 * This class provides a familiar API while leveraging CTRE's optimized
 * odometry thread and SwerveRequest-based control.
 */
public class SwerveSubsystem extends SubsystemBase {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

//region Members & constructor -------------------------------------------------

    final CommandSwerveDrivetrain drivetrain;
    final List<Consumer<Pose2d>> poseResetListeners;
    final Field2d field2d;

    // Reusable SwerveRequest objects
    final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();

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
        this.poseResetListeners = new ArrayList<>();
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
        // CTRE drivetrain handles odometry in its own thread
        // We just need to publish visualization data
        Pose2d pose = getPose();
        field2d.setRobotPose(pose);
        Util.publishPose("Fused", pose);

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
        // In simulation, manually integrate commanded speeds to update odometry
        // since CTRE simulation may not fully update the pose estimator
        boolean hasMovement = latestSpeed != null &&
            (Math.abs(latestSpeed.vxMetersPerSecond) > 0.001 ||
             Math.abs(latestSpeed.vyMetersPerSecond) > 0.001 ||
             Math.abs(latestSpeed.omegaRadiansPerSecond) > 0.001);
        if (hasMovement) {
            Pose2d currentPose = getPose();

            // integrate speeds over one loop period (20ms)
            double dt = Util.DT;
            double dx = latestSpeed.vxMetersPerSecond * dt;
            double dy = latestSpeed.vyMetersPerSecond * dt;
            double dtheta = latestSpeed.omegaRadiansPerSecond * dt;

            // convert robot-relative speeds to field-relative
            Rotation2d heading = currentPose.getRotation();
            double fieldDx = dx * heading.getCos() - dy * heading.getSin();
            double fieldDy = dx * heading.getSin() + dy * heading.getCos();

            // create new pose
            Pose2d newPose = new Pose2d(
                currentPose.getX() + fieldDx,
                currentPose.getY() + fieldDy,
                currentPose.getRotation().plus(Rotation2d.fromRadians(dtheta))
            );

            // update the drivetrain's pose (bypass normal reset to avoid notifications)
            drivetrain.resetPose(newPose);
        }
    }

//endregion

//region Pose accessors --------------------------------------------------------

    /**
     * @return the current robot heading from the gyro
     */
    public Rotation2d getHeading() {
        return drivetrain.getState().Pose.getRotation();
    }

    /**
     * @return the current estimated pose (odometry fused with vision)
     */
    public Pose2d getPose() {
        return drivetrain.getState().Pose;
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

        // notify listeners
        for (Consumer<Pose2d> listener : poseResetListeners) {
            listener.accept(newPose);
        }

        // increment counter for dashboard
        resetCount++;

        Util.log("[swerve] reset pose to %s", newPose);
    }

    /**
     * Registers a listener to be notified when the pose is reset.
     *
     * @param listener the listener to register
     */
    public void addPoseResetListener(Consumer<Pose2d> listener) {
        poseResetListeners.add(listener);
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
                .withCenterOfRotation(center)
        );
    }

    /**
     * Drives the robot using field-relative speeds.
     *
     * @param mode the mode to display for debugging
     * @param speeds field-relative chassis speeds (vx, vy in m/s, omega in rad/s)
     */
    public void driveFieldRelative(String mode, ChassisSpeeds speeds) {
        currentMode = mode;
        latestSpeed = speeds;

        drivetrain.setControl(
            fieldCentricRequest
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
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
     * @return a command that resets all wheel angles to 0 (forward facing)
     */
    public Command resetWheelsCommand() {
        return runOnce(() -> {
            drivetrain.setControl(pointRequest.withModuleDirection(Rotation2d.kZero));
            wheelsLocked = true;
            Util.log("Wheels reset to forward");
        }).finallyDo(() -> wheelsLocked = false);
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
     * Creates a command that drives at fixed speeds.
     *
     * @param speeds the speeds to drive at
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
     * Creates a command that rotates to face a heading dynamically
     * re-calculated whenever the command executes.
     *
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
     * Creates a command that drives to the specified pose along a straight line.
     *
     * @param pose the target pose
     * @return the pose command
     * @see SwerveToPoseCommand
     */
    public Command driveToPoseCommand(Pose2d pose) {
        return new SwerveToPoseCommand(this, pose);
    }

    /**
     * Creates a command that drives to a pose dynamically re-calculated
     * whenever the command executes.
     *
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

    /**
     * @return the underlying CommandSwerveDrivetrain for direct access
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

//endregion
}

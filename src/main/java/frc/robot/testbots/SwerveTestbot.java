package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameController;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;

/**
 * Standalone test program for the swerve drive subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=SwerveTestbot}
 */
public class SwerveTestbot extends TimedRobot {

    final SwerveSubsystem swerve;
    final LimelightSubsystem limelight;
    final GameController controller;

    public SwerveTestbot() {

        // CTRE drivetrain handles both sim and real hardware automatically
        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        limelight = new LimelightSubsystem(swerve);

        controller = new GameController(0);

        // default to field-relative driving with turbo/sniper modes
        swerve.setDefaultCommand(swerve.driveCommand(controller));

        // Odometry reset on start
        controller.start().onTrue(swerve.zeroPoseCommand());

        // a will drive the robot one meter straight forward from its
        // current position
        controller.a().onTrue(swerve.defer(() -> {
            Pose2d currentPose = swerve.getPose();
            Pose2d newPose = currentPose.transformBy(new Transform2d(
                    1.0,
                    0.0,
                    Rotation2d.kZero));
            return swerve.driveToPoseCommand(newPose);
        }));

        // b will drive the robot one meter straight forward, turn it around,
        // drive it back and then turn around again to start position
        controller.b().onTrue(swerve.defer(() -> {

            Pose2d p0 = swerve.getPose();
            Rotation2d h0 = swerve.getHeading();

            Pose2d p1 = p0.transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero));
            Rotation2d h1 = h0.plus(Rotation2d.k180deg);

            return Commands.sequence(
                swerve.driveToPoseCommand(p1),
                swerve.driveToHeadingCommand(h1),
                swerve.driveToPoseCommand(new Pose2d(p0.getTranslation(), h1)),
                swerve.driveToHeadingCommand(h0));
        }));

        // x will drive the robot around a square one meter on a side, to
        // the left
        controller.x().onTrue(swerve.defer(() -> {

            Pose2d p0 = swerve.getPose();
            Pose2d p1 = p0.transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero));
            Pose2d p2 = p1.transformBy(new Transform2d(0.0, 1.0, Rotation2d.kZero));
            Pose2d p3 = p2.transformBy(new Transform2d(-1.0, 0.0, Rotation2d.kZero));

            return Commands.sequence(
                    swerve.driveToPoseCommand(p1),
                    swerve.driveToPoseCommand(p2),
                    swerve.driveToPoseCommand(p3),
                    swerve.driveToPoseCommand(p0));
        }));

        controller.rightBumper()
                .onTrue(limelight.resetPoseFromVisionCommand());

    }

    @Override
    public void robotInit() {
        System.out.println(">>> SwerveTestbot starting...");
        System.out.println(">>> Button bindings configured - press A, B, X, Y, Start, Back, etc.");

        // debug controller inputs
        SmartDashboard.putData("Controller", builder -> {
            builder.addBooleanProperty("LeftBumper", () -> controller.getHID().getRawButton(5), null);
            builder.addBooleanProperty("RightBumper", () -> controller.getHID().getRawButton(6), null);
            builder.addDoubleProperty("LeftX", controller::getLeftX, null);
            builder.addDoubleProperty("LeftY", controller::getLeftY, null);
            builder.addDoubleProperty("RightX", controller::getRightX, null);
            builder.addDoubleProperty("LeftTrigger", controller::getLeftTriggerAxis, null);
            builder.addDoubleProperty("RightTrigger", controller::getRightTriggerAxis, null);
        });

        // debug CTRE pose data directly
        SmartDashboard.putData("CTRE_Pose", builder -> {
            builder.addDoubleProperty("X_meters", () -> swerve.getPose().getX(), null);
            builder.addDoubleProperty("Y_meters", () -> swerve.getPose().getY(), null);
            builder.addDoubleProperty("X_feet", () -> swerve.getPose().getX() * 3.28084, null);
            builder.addDoubleProperty("Y_feet", () -> swerve.getPose().getY() * 3.28084, null);
            builder.addDoubleProperty("Heading_deg", () -> swerve.getHeading().getDegrees(), null);
            builder.addDoubleProperty("Speeds_vx", () -> swerve.getCurrentSpeeds().vxMetersPerSecond, null);
            builder.addDoubleProperty("Speeds_vy", () -> swerve.getCurrentSpeeds().vyMetersPerSecond, null);
            builder.addDoubleProperty("Speeds_omega", () -> Math.toDegrees(swerve.getCurrentSpeeds().omegaRadiansPerSecond), null);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

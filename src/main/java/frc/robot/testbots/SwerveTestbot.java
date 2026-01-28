package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveHardwareCTRE;
import frc.robot.subsystems.swerve.SwerveHardwareSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Standalone test program for the swerve drive subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=SwerveTestbot}
 */
public class SwerveTestbot extends TimedRobot {

    final SwerveSubsystem swerve;
    final GameController controller;

    public SwerveTestbot() {

        swerve = new SwerveSubsystem(isSimulation() ?
                new SwerveHardwareSim() :
                new SwerveHardwareCTRE());

        controller = new GameController(0);

        // default to field-relative driving with turbo/sniper modes
        swerve.setDefaultCommand(swerve.driveCommand(controller));

        // Odometry reset on start
        controller.start().onTrue(swerve.zeroPoseCommand());

        // teleport buttons for driving to different field positions
        controller.a().onTrue(swerve.driveToPoseCommand(oldPose -> new Pose2d(
                2.0,
                2.0,
                Rotation2d.kZero)));
        controller.b().onTrue(swerve.driveToPoseCommand(oldPose -> new Pose2d(
                8.0,
                4.0,
                Rotation2d.k180deg)));
        controller.x().onTrue(swerve.driveToPoseCommand(oldPose -> new Pose2d(
                14.0,
                6.0,
                Rotation2d.kCCW_Pi_2)));

        // Orbit mode: hold left bumper to orbit around the Reef while facing it
        controller.leftBumper()
                .whileTrue(swerve.orbitCommand(controller));

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
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

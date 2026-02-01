package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

        // Odometry reset on start
        controller.back().whileTrue(swerve.resetWheelsCommand());

        // a will drive the robot one meter straight forward from its
        // current position
        // controller.a().onTrue(swerve.driveToPoseCommand(oldPose ->
        //     oldPose.transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero))));
        controller.a().onTrue(swerve.fixedSpeedCommand(new ChassisSpeeds(0.5, 0.0, 0.0), 2.0));
        
        controller.leftBumper().onTrue(swerve.zeroPoseCommand());

        // // b will drive the robot one meter straight forward, turn it around,
        // // drive it back and then turn around again to start position
        // controller.b().onTrue(swerve.defer(() -> {

        //     Pose2d p0 = swerve.getPose();
        //     Rotation2d h0 = swerve.getHeading();

        //     Pose2d p1 = p0.transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero));
        //     Rotation2d h1 = h0.plus(Rotation2d.k180deg);

        //     return Commands.sequence(
        //         swerve.driveToPoseCommand(p1),
        //         swerve.driveToHeadingCommand(h1),
        //         swerve.driveToPoseCommand(new Pose2d(p0.getTranslation(), h1)),
        //         swerve.driveToHeadingCommand(h0));
        // }));

        // // x will drive the robot around a square one meter on a side, to
        // // the left
        // controller.x().onTrue(swerve.defer(() -> {

        //     Pose2d p0 = swerve.getPose();
        //     Pose2d p1 = p0.transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero));
        //     Pose2d p2 = p1.transformBy(new Transform2d(0.0, 1.0, Rotation2d.kZero));
        //     Pose2d p3 = p2.transformBy(new Transform2d(-1.0, 0.0, Rotation2d.kZero));

        //     return Commands.sequence(
        //             swerve.driveToPoseCommand(p1),
        //             swerve.driveToPoseCommand(p2),
        //             swerve.driveToPoseCommand(p3),
        //             swerve.driveToPoseCommand(p0));
        // }));

        // // Orbit mode: hold left bumper to orbit around the Reef while facing it
        // controller.leftBumper()
        //         .whileTrue(swerve.orbitCommand(controller));

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

package frc.robot.testbots;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSim;

/**
 * Standalone test program for the vision simulation.
 * <p>
 * Tests the LimelightSim by driving around and observing:
 * <ul>
 *   <li>Tag detection based on position and orientation</li>
 *   <li>tx/ty angles changing as robot moves</li>
 *   <li>Vision pose estimates being accepted/rejected</li>
 * </ul>
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=VisionSimTestbot}
 */
public class VisionSimTestbot extends TimedRobot {

    private static final double DEADZONE = 0.05;

    private SwerveSubsystem swerve;
    private LimelightSim limelightSim;
    private CommandXboxController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> VisionSimTestbot starting...");

        // create swerve with simulation hardware
        swerve = new SwerveSubsystem(new SwerveSim());
        controller = new CommandXboxController(0);

        // create limelight simulation
        limelightSim = new LimelightSim();

        System.out.println(">>> Vision simulation initialized");
        System.out.println(">>> Drive around to test tag detection");
        System.out.println(">>> Press START to zero heading");
        System.out.println(">>> Press BACK to reset pose to origin");
        System.out.println(">>> Press A to teleport to (2, 2)");
        System.out.println(">>> Press B to teleport to (8, 4) - field center");
        System.out.println(">>> Press Y to teleport to (14, 6)");

        // default to field-relative driving
        swerve.setDefaultCommand(swerve.driveCommand(
                () -> -MathUtil.applyDeadband(controller.getRawAxis(1), DEADZONE),
                () -> -MathUtil.applyDeadband(controller.getRawAxis(0), DEADZONE),
                () -> -MathUtil.applyDeadband(controller.getRawAxis(2), DEADZONE),
                true));

        // button bindings
        controller.start().onTrue(Commands.runOnce(() -> {
            swerve.zeroHeading();
            System.out.println(">>> Gyro zeroed");
        }));

        controller.back().onTrue(Commands.runOnce(() -> {
            swerve.zeroHeading();
            swerve.resetPose(new Pose2d());
            System.out.println(">>> Pose reset to origin");
        }));

        // teleport buttons for testing different field positions
        controller.a().onTrue(Commands.runOnce(() -> {
            teleportTo(2.0, 2.0, 0);
            System.out.println(">>> Teleported to (2, 2)");
        }));

        controller.b().onTrue(Commands.runOnce(() -> {
            teleportTo(8.0, 4.0, 0);
            System.out.println(">>> Teleported to field center (8, 4)");
        }));

        controller.y().onTrue(Commands.runOnce(() -> {
            teleportTo(14.0, 6.0, 180);
            System.out.println(">>> Teleported to (14, 6) facing left");
        }));

        // X to lock wheels
        controller.x().whileTrue(swerve.idleCommand());

        // dashboard telemetry for vision testing
        SmartDashboard.putData("VisionTest", builder -> {
            builder.addDoubleProperty("PoseX", () -> swerve.getPose().getX(), null);
            builder.addDoubleProperty("PoseY", () -> swerve.getPose().getY(), null);
            builder.addDoubleProperty("HeadingDeg", () -> swerve.getHeading().getDegrees(), null);
            builder.addBooleanProperty("VisionValid", swerve::isVisionValid, null);
        });

        System.out.println(">>> VisionSimTestbot ready!");
    }

    /**
     * Teleports the robot to a specific position.
     */
    private void teleportTo(double x, double y, double headingDeg) {
        Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
        swerve.resetPose(newPose);
    }

    @Override
    public void robotPeriodic() {
        // update limelight simulation BEFORE scheduler runs
        // this ensures vision data is available for pose estimation
        limelightSim.update(swerve.getPose());

        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        System.out.println(">>> Teleop enabled - drive around to test vision!");
    }

    @Override
    public void autonomousInit() {
        System.out.println(">>> Autonomous enabled");
    }
}

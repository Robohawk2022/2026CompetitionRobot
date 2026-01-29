package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.limelight.LimelightSim;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.SwerveHardwareSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Standalone test program for the vision simulation. Tests the LimelightSim
 * by driving around and observing:
 * <ul>
 *   <li>Tag detection based on position and orientation</li>
 *   <li>tx/ty angles changing as robot moves</li>
 *   <li>Vision pose estimates being accepted/rejected</li>
 * </ul>
 * Run with: {@code ./gradlew simulateJava -Probot=VisionSimTestbot}
 */
public class VisionSimTestbot extends TimedRobot {

    final SwerveSubsystem swerve;
    final LimelightSim limelightSim;
    final LimelightSubsystem limelightSubsystem;
    final GameController controller;

    public VisionSimTestbot() {

        swerve = new SwerveSubsystem(new SwerveHardwareSim());
        limelightSim = new LimelightSim();
        limelightSubsystem = new LimelightSubsystem(swerve);
        controller = new GameController(0);

        // default to field-relative driving
        swerve.setDefaultCommand(swerve.driveCommand(controller));

        // start zeroes heading but keeps the robot where it is
        controller.start().onTrue(swerve.zeroHeadingCommand());

        // back zeroes pose entirely
        controller.back().onTrue(swerve.zeroPoseCommand());

        // teleport buttons for testing different field positions
        controller.a().onTrue(swerve.resetPoseCommand(oldPose -> new Pose2d(
                2.0,
                2.0,
                Rotation2d.kZero)));
        controller.b().onTrue(swerve.resetPoseCommand(oldPose -> new Pose2d(
                8.0,
                4.0,
                Rotation2d.kZero)));
        controller.x().onTrue(swerve.resetPoseCommand(oldPose -> new Pose2d(
                14.0,
                6.0,
                Rotation2d.k180deg)));

        // Y to lock wheels
        controller.y().whileTrue(swerve.lockWheelsCommand());
    }

    @Override
    public void robotInit() {
        System.out.println(">>> VisionSimTestbot starting...");
        System.out.println(">>> Vision simulation initialized");
        System.out.println(">>> Drive around to test tag detection");
        System.out.println(">>> Press START to zero heading");
        System.out.println(">>> Press BACK to reset pose to origin");
        System.out.println(">>> Press A to teleport to (2, 2)");
        System.out.println(">>> Press B to teleport to (8, 4) - field center");
        System.out.println(">>> Press Y to teleport to (14, 6)");
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

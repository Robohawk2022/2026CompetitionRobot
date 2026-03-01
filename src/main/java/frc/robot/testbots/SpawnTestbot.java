package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;

/**
 * Standalone test program that spawns the robot at a specific field position.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=SpawnTestbot}
 */
public class SpawnTestbot extends TimedRobot {

    static final Pose2d SPAWN_POSE = new Pose2d(11.916, 4.035, Rotation2d.kZero);

    final SwerveSubsystem swerve;
    final LimelightSubsystem limelight;
    final GameController controller;

    public SpawnTestbot() {

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        limelight = new LimelightSubsystem(swerve);
        controller = new GameController(0);

        // teleport to spawn position
        swerve.resetPose(SPAWN_POSE);

        // default to field-relative driving
        swerve.setDefaultCommand(swerve.teleopCommand(controller));

        // start zeroes heading but keeps robot position
        controller.start().onTrue(swerve.zeroHeadingCommand());

        // back resets pose to spawn position
        controller.back().onTrue(swerve.resetPoseCommand(oldPose -> SPAWN_POSE));

        // right bumper resets pose from vision
        controller.rightBumper().onTrue(limelight.resetPoseFromVisionCommand());
    }

    @Override
    public void robotInit() {
        System.out.println(">>> SpawnTestbot starting...");
        System.out.println(">>> Spawning at (11.916, 4.035) meters");
        System.out.println(">>> Press START to zero heading");
        System.out.println(">>> Press BACK to teleport back to spawn");

        SmartDashboard.putData("Pose", builder -> {
            builder.addDoubleProperty("X_meters", () -> swerve.getPose().getX(), null);
            builder.addDoubleProperty("Y_meters", () -> swerve.getPose().getY(), null);
            builder.addDoubleProperty("X_feet", () -> swerve.getPose().getX() * 3.28084, null);
            builder.addDoubleProperty("Y_feet", () -> swerve.getPose().getY() * 3.28084, null);
            builder.addDoubleProperty("Heading_deg", () -> swerve.getHeading().getDegrees(), null);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

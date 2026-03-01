package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.ballpath.BallPathHardwareSim;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterHardwareSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.Field;

/**
 * Standalone test program that spawns the robot 8 feet from the hub center.
 * <p>
 * Press A to teleport to the +45 degree position, B for -45 degrees.
 * Both positions face the hub.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=SpawnTestbot}
 */
public class SpawnTestbot extends TimedRobot {

    static final double DISTANCE_FEET = 8.0;

    final SwerveSubsystem swerve;
    final ShooterSubsystem shooter;
    final BallPathSubsystem ballPath;
    final LEDSubsystem led;
    final LimelightSubsystem limelight;
    final GameController controller;

    public SpawnTestbot() {

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        shooter = new ShooterSubsystem(new ShooterHardwareSim());
        ballPath = new BallPathSubsystem(new BallPathHardwareSim());
        led = new LEDSubsystem(new LEDHardwareSim());
        limelight = new LimelightSubsystem(swerve);
        controller = new GameController(0);

        // spawn at the +45 degree position by default
        swerve.resetPose(poseFromHub(45));

        // default to field-relative driving
        swerve.setDefaultCommand(swerve.teleopCommand(controller));
        shooter.setDefaultCommand(shooter.coast());
        ballPath.setDefaultCommand(ballPath.coast());

        // A teleports to +45 degrees from hub, facing hub
        controller.a().onTrue(swerve.resetPoseCommand(oldPose -> poseFromHub(45)));

        // B teleports to -45 degrees from hub, facing hub
        controller.b().onTrue(swerve.resetPoseCommand(oldPose -> poseFromHub(-45)));

        // X runs the shoot auto (drive to hub and shoot)
        controller.x().whileTrue(ShootingCommands.driveAndShootCommand(led, swerve, shooter, ballPath));

        // Y runs orient-only (drive to 8ft from hub without shooting)
        controller.y().whileTrue(ShootingCommands.orientToShoot(led, swerve));

        // start zeroes heading but keeps robot position
        controller.start().onTrue(swerve.zeroHeadingCommand());

        // back resets pose to origin
        controller.back().onTrue(swerve.zeroPoseCommand());

        // right bumper resets pose from vision
        controller.rightBumper().onTrue(limelight.resetPoseFromVisionCommand());
    }

    /**
     * Computes a pose that is 8 feet from the hub center at the given angle,
     * facing the hub.
     *
     * @param angleDeg angle from hub center in degrees (0 = toward red wall)
     * @return the pose facing the hub
     */
    static Pose2d poseFromHub(double angleDeg) {
        Pose2d hub = Field.getHubCenter();
        double distMeters = Units.feetToMeters(DISTANCE_FEET);
        double angleRad = Math.toRadians(angleDeg);

        // position 8 feet out from hub at the given angle
        Translation2d offset = new Translation2d(
                distMeters * Math.cos(angleRad),
                distMeters * Math.sin(angleRad));
        Translation2d position = hub.getTranslation().plus(offset);

        // face back toward the hub
        Rotation2d heading = offset.getAngle().plus(Rotation2d.k180deg);

        return new Pose2d(position, heading);
    }

    @Override
    public void robotInit() {
        Pose2d hub = Field.getHubCenter();
        System.out.println(">>> SpawnTestbot starting...");
        System.out.printf(">>> Hub center at (%.2f, %.2f) meters%n", hub.getX(), hub.getY());
        System.out.println(">>> Spawning 8 feet from hub at +45 degrees");
        System.out.println(">>> Press A to teleport to +45 deg position");
        System.out.println(">>> Press B to teleport to -45 deg position");
        System.out.println(">>> Hold X to run shoot auto (drive + shoot)");
        System.out.println(">>> Hold Y to orient only (drive to 8ft from hub)");
        System.out.println(">>> Press START to zero heading");
        System.out.println(">>> Press BACK to reset pose to origin");

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

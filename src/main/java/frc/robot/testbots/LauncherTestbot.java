package frc.robot.testbots;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ballpath.BallPathHardwareRev;
import frc.robot.subsystems.ballpath.BallPathHardwareSim;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.shooter.ShooterHardwareRev;
import frc.robot.subsystems.shooter.ShooterHardwareSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Standalone test program for the Shooter + BallPath subsystems.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LauncherTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake (ball-path motors spin inward)</li>
 *   <li>B (hold) - Eject (ball-path motors spin outward)</li>
 *   <li>X (hold) - Shoot (shooter spins up + ball-path feeds)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 */
public class LauncherTestbot extends TimedRobot {

    // CAN IDs from the testing setup
    public static final int INTAKE_CAN_ID = RobotContainer.INTAKE_CAN_ID; // 9;
    public static final int FEEDER_CAN_ID = RobotContainer.FEEDER_CAN_ID; // 60;
    public static final int AGITATOR_CAN_ID = RobotContainer.AGITATOR_CAN_ID; // 2;
    public static final int SHOOTER_CAN_ID = RobotContainer.SHOOTER_CAN_ID; // 35;

    private ShooterSubsystem shooter;
    private BallPathSubsystem ballPath;
    private double testRpm;

    @Override
    public void robotInit() {

        testRpm = 1000.0;

        System.out.println(">>> LauncherTestbot starting...");

        // clear stale Preferences so code defaults always win on deploy
        Preferences.removeAll();

        shooter = new ShooterSubsystem(isSimulation()
                ? new ShooterHardwareSim()
                : new ShooterHardwareRev(SHOOTER_CAN_ID));

        ballPath = new BallPathSubsystem(isSimulation()
                ? new BallPathHardwareSim()
                : new BallPathHardwareRev(INTAKE_CAN_ID, FEEDER_CAN_ID, AGITATOR_CAN_ID));

        GameController controller = new GameController(0);

        shooter.setDefaultCommand(shooter.coast());
        ballPath.setDefaultCommand(ballPath.coast());

        // intake: ball-path motors spin inward
        controller.a().whileTrue(ballPath.intakeCommand());

        // eject: ball-path motors spin outward
        controller.b().whileTrue(ballPath.ejectCommand());

        // shoot: shooter spins up + ball-path feeds
        controller.x().whileTrue(Commands.parallel(
                shooter.shootCommand(),
                ballPath.feedCommand()));

        // stop all motors
        controller.y().onTrue(Commands.parallel(
                shooter.coast(),
                ballPath.coast()));

        // left bumper: run one motor at test velocity (uncomment the one you want)
        controller.leftBumper().whileTrue(
                ballPath.defer(() -> ballPath.velocityCommand(testRpm, 0.0, 0.0)));
                // ballPath.defer(() -> ballPath.velocityCommand(0.0, testRpm, 0.0)));
                // ballPath.defer(() -> ballPath.velocityCommand(0.0, 0.0, testRpm)));
                // shooter.defer(() -> shooter.velocityCommand(testRpm)));

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (ball-path inward)");
        System.out.println("    B (hold) = Eject (ball-path outward)");
        System.out.println("    X (hold) = Shoot (shooter + feed)");

        SmartDashboard.putData("LauncherTestbot", builder -> {
            builder.addDoubleProperty("TestRpm", () -> testRpm, val -> testRpm = val);
        });

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        shooter.coast();
        ballPath.coast();
    }
}

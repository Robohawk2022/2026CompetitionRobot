package frc.robot.testbots;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Standalone test program for the Launcher subsystem (3-motor design).
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LauncherTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake (feeders spin inward)</li>
 *   <li>B (hold) - Eject (feeders spin outward)</li>
 *   <li>X (hold) - Shoot (feeders + shooter spin up)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 */
public class LauncherTestbot extends TimedRobot {

    public static final int FEEDER_LEFT_CAN_ID = 32;
    public static final int FEEDER_RIGHT_CAN_ID = 11;
    public static final int SHOOTER_CAN_ID = 60;

    private LauncherSubsystem launcher;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> LauncherTestbot starting...");

        // clear stale Preferences so code defaults always win on deploy
        Preferences.removeAll();

        boolean sim = isSimulation();
        launcher = new LauncherSubsystem(sim
                ? new LauncherHardwareSim()
                : new LauncherHardwareRev(
                        FEEDER_LEFT_CAN_ID,
                        FEEDER_RIGHT_CAN_ID,
                        SHOOTER_CAN_ID));
        controller = new GameController(0);

        launcher.setDefaultCommand(launcher.idleCommand());

        // intake: feeders spin inward
        controller.a().whileTrue(launcher.intakeCommand());

        // eject: feeders spin outward
        controller.b().whileTrue(launcher.ejectCommand());

        // shoot: feeders + shooter spin up
        controller.x().whileTrue(launcher.shootCommand());

        // stop all
        controller.y().onTrue(launcher.stopCommand());

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (feeders inward)");
        System.out.println("    B (hold) = Eject (feeders outward)");
        System.out.println("    X (hold) = Shoot (feeders + shooter)");
        System.out.println("    Y (press) = Stop all");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        launcher.stop();
    }
}

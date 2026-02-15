package frc.robot.testbots;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameController;
import frc.robot.subsystems.agitator.AgitatorHardwareRev;
import frc.robot.subsystems.agitator.AgitatorHardwareSim;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem.ShotPreset;

/**
 * Standalone test program for the Launcher + Agitator subsystems.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LauncherTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake (lower wheel backward + agitator forward)</li>
 *   <li>B (hold) - Eject (lower wheel forward + agitator reversed)</li>
 *   <li>X (hold) - Shoot NEUTRAL (spin up wheels, wait for speed, then feed)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 */
public class LauncherTestbot extends TimedRobot {

    private LauncherSubsystem launcher;
    private AgitatorSubsystem agitator;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> LauncherTestbot starting...");

        boolean sim = isSimulation();
        launcher = new LauncherSubsystem(sim ? new LauncherHardwareSim() : new LauncherHardwareRev());
        agitator = new AgitatorSubsystem(sim ? new AgitatorHardwareSim() : new AgitatorHardwareRev());
        controller = new GameController(0);

        launcher.setDefaultCommand(launcher.idleCommand());
        agitator.setDefaultCommand(agitator.idleCommand());

        // intake: lower wheel backward + agitator forward (parallel)
        controller.a().whileTrue(Commands.parallel(
                launcher.intakeWheelCommand(),
                agitator.forwardCommand()));

        // eject: lower wheel forward + agitator reversed (parallel)
        controller.b().whileTrue(Commands.parallel(
                launcher.ejectWheelCommand(),
                agitator.reverseCommand()));

        // shoot: spin up wheels, wait for speed, then start feeding
        controller.x().whileTrue(shootCommand(ShotPreset.NEUTRAL));

        // stop all
        controller.y().onTrue(Commands.parallel(
                launcher.stopCommand(),
                agitator.stopCommand()));

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (lower wheel backward + agitator forward)");
        System.out.println("    B (hold) = Eject (lower wheel forward + agitator reversed)");
        System.out.println("    X (hold) = Shoot NEUTRAL (spin up, wait, then feed)");
        System.out.println("    Y (press) = Stop all");
    }

    /**
     * Shoot command: spins up wheels and starts agitator once at speed.
     * Wheels keep spinning the entire time — no gap between spin-up and feed.
     */
    private Command shootCommand(ShotPreset preset) {
        return Commands.parallel(
            launcher.spinUpCommand(preset),
            Commands.sequence(
                Commands.waitUntil(launcher::atSpeed).withTimeout(2.0),
                agitator.feedCommand()
            )
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        launcher.stop();
        agitator.stop();
    }
}

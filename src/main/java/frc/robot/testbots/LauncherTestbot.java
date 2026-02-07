package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem.ShotPreset;

/**
 * Standalone test program for the Launcher subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LauncherTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake (pulls balls in, agitator feeds)</li>
 *   <li>B (hold) - Eject (reverses intake and agitator)</li>
 *   <li>X (hold) - Shoot HIGH ARC (upper wheel faster = backspin)</li>
 *   <li>Y (hold) - Shoot FLAT (lower wheel faster = topspin)</li>
 *   <li>Right bumper (hold) - Shoot NEUTRAL (equal wheel speed)</li>
 *   <li>Left bumper (hold) - Spin up NEUTRAL (no feed, just wheels)</li>
 * </ul>
 * <p>
 * <b>What to watch on the dashboard:</b>
 * <ul>
 *   <li>LauncherSubsystem/Mode - current operating mode</li>
 *   <li>LauncherSubsystem/LowerWheelRPM - lower wheel speed</li>
 *   <li>LauncherSubsystem/UpperWheelRPM - upper wheel speed</li>
 *   <li>For HIGH_ARC: UpperWheelRPM should be higher than LowerWheelRPM</li>
 *   <li>For FLAT: LowerWheelRPM should be higher than UpperWheelRPM</li>
 * </ul>
 */
public class LauncherTestbot extends TimedRobot {

    private LauncherSubsystem launcher;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> LauncherTestbot starting...");

        launcher = new LauncherSubsystem(
                isSimulation() ? new LauncherHardwareSim() : new LauncherHardwareRev());
        controller = new GameController(0);

        launcher.setDefaultCommand(launcher.idleCommand());

        // intake / eject
        controller.a().whileTrue(launcher.intakeCommand());
        controller.b().whileTrue(launcher.ejectCommand());

        // shoot with different arcs
        controller.x().whileTrue(launcher.shootCommand(ShotPreset.HIGH_ARC));
        controller.y().whileTrue(launcher.shootCommand(ShotPreset.FLAT));
        controller.rightBumper().whileTrue(launcher.shootCommand(ShotPreset.NEUTRAL));

        // spin up without feeding (pre-spin)
        controller.leftBumper().whileTrue(launcher.spinUpCommand(ShotPreset.NEUTRAL));

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (pull balls in)");
        System.out.println("    B (hold) = Eject (push balls out)");
        System.out.println("    X (hold) = Shoot HIGH ARC (backspin)");
        System.out.println("    Y (hold) = Shoot FLAT (topspin)");
        System.out.println("    RB (hold) = Shoot NEUTRAL (equal speed)");
        System.out.println("    LB (hold) = Spin up only (no feed)");
        System.out.println("");
        System.out.println(">>> Tune via Shuffleboard:");
        System.out.println("    Launcher/HighArc/LowerPower% and UpperPower%");
        System.out.println("    Launcher/Flat/LowerPower% and UpperPower%");
        System.out.println("    Launcher/Neutral/Power%");
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

package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem.ShotPreset;

/**
 * Standalone test program for the Launcher subsystem (3-motor design).
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LauncherTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake (lower wheel backward + agitator feeds)</li>
 *   <li>B (hold) - Eject (lower wheel backward + agitator reversed)</li>
 *   <li>X (hold) - Shoot NEUTRAL (equal wheel speed + agitator feeds)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 * <p>
 * <b>What to watch on the dashboard:</b>
 * <ul>
 *   <li>LauncherSubsystem/Mode - current operating mode</li>
 *   <li>LauncherSubsystem/LowerWheelCurrent - lower wheel RPM (negative when intaking)</li>
 *   <li>LauncherSubsystem/UpperWheelCurrent - upper wheel RPM</li>
 *   <li>LauncherSubsystem/AtSpeed? - true when wheels are at target RPM</li>
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

        // intake / eject (lower wheel backward)
        controller.a().whileTrue(launcher.intakeCommand());
        controller.b().whileTrue(launcher.ejectCommand());

        // shoot neutral preset
        controller.x().whileTrue(launcher.shootCommand(ShotPreset.NEUTRAL));

        // stop all
        controller.y().onTrue(launcher.stopCommand());

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (lower wheel backward + agitator)");
        System.out.println("    B (hold) = Eject (lower wheel backward + agitator reversed)");
        System.out.println("    X (hold) = Shoot NEUTRAL (both wheels + agitator feeds)");
        System.out.println("    Y (press) = Stop all");
        System.out.println("");
        System.out.println(">>> Tune via Shuffleboard:");
        System.out.println("    Launcher/IntakeSpeedRPM - intake speed for lower wheel");
        System.out.println("    Launcher/Wheels/kV, kP, kD - velocity PID tuning");
        System.out.println("    Launcher/Neutral/RPM - neutral shot speed");
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

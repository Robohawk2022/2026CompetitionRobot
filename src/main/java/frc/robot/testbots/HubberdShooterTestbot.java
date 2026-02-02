package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.hubberdshooter.HubbardShooterHardwareRev;
import frc.robot.subsystems.hubberdshooter.HubberdShooterHardwareSim;
import frc.robot.subsystems.hubberdshooter.HubberdShooterSubsystem;

/**
 * Standalone test program for the HubberdShooter subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=HubberdShooterTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake mode (both motors pull fuel in at 20%)</li>
 *   <li>B (hold) - Outtake mode (both motors push fuel out at 20%)</li>
 *   <li>X (hold) - Shoot mode (motors counter-rotate at 1200 RPM)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 * <p>
 * <b>Configuration (via Elastic/Shuffleboard):</b>
 * <ul>
 *   <li>{@code HubberdShooter/IntakePower%} - Power for intake mode (0-100%)</li>
 *   <li>{@code HubberdShooter/OuttakePower%} - Power for outtake mode (0-100%)</li>
 *   <li>{@code HubberdShooter/ShootingTargetRPM} - Target RPM for shooting mode</li>
 *   <li>{@code HubberdShooter/kV} - Velocity feedforward gain</li>
 *   <li>{@code HubberdShooter/kP} - Proportional gain</li>
 *   <li>{@code HubberdShooter/kS} - Static friction compensation</li>
 *   <li>{@code HubberdShooter/Motor1Inverted?} - Invert motor 1</li>
 *   <li>{@code HubberdShooter/Motor2Inverted?} - Invert motor 2 (default true for counter-rotation)</li>
 * </ul>
 * <p>
 * <b>Expected behavior:</b>
 * <ul>
 *   <li>Intake (A): Both motors show positive RPM on dashboard</li>
 *   <li>Outtake (B): Both motors show negative RPM on dashboard</li>
 *   <li>Shoot (X): Motor1 positive ~1200 RPM, Motor2 negative ~1200 RPM</li>
 * </ul>
 */
public class HubberdShooterTestbot extends TimedRobot {

    private HubberdShooterSubsystem shooter;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> HubberdShooterTestbot starting...");

        // Use appropriate hardware based on environment
        shooter = new HubberdShooterSubsystem(
                isSimulation() ? new HubberdShooterHardwareSim() : new HubbardShooterHardwareRev());
        controller = new GameController(0);

        // Set default command to idle
        shooter.setDefaultCommand(shooter.idleCommand());

        // Button bindings
        controller.a().whileTrue(shooter.intakeCommand());
        controller.b().whileTrue(shooter.outtakeCommand());
        controller.x().whileTrue(shooter.shootCommand());
        controller.y().onTrue(shooter.stopCommand());

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (both motors pull fuel in)");
        System.out.println("    B (hold) = Outtake (both motors push fuel out)");
        System.out.println("    X (hold) = Shoot (motors counter-rotate at target RPM)");
        System.out.println("    Y (press) = Stop all motors");
        System.out.println("");
        System.out.println(">>> Configuration:");
        System.out.println("    HubberdShooter/IntakePower% = Power for intake mode");
        System.out.println("    HubberdShooter/OuttakePower% = Power for outtake mode");
        System.out.println("    HubberdShooter/ShootingTargetRPM = Target RPM for shooting");
        System.out.println("    HubberdShooter/kV = Velocity feedforward gain");
        System.out.println("    HubberdShooter/kP = Proportional gain");
        System.out.println("    HubberdShooter/kS = Static friction compensation");
        System.out.println("");
        System.out.println(">>> Expected behavior:");
        System.out.println("    Intake (A): Both motors positive RPM");
        System.out.println("    Outtake (B): Both motors negative RPM");
        System.out.println("    Shoot (X): Motor1 +1200 RPM, Motor2 -1200 RPM (counter-rotate)");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Stop motors immediately when robot is disabled
        shooter.stop();
    }
}

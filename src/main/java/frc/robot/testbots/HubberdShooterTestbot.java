package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.hubberdshooter.HubberdShooterHardwareCTRE;
import frc.robot.subsystems.hubberdshooter.HubberdShooterHardwareSim;
import frc.robot.subsystems.hubberdshooter.HubberdShooterSubsystem;

/**
 * Standalone test program for the HubberdShooter subsystem.
 * <p>
 * Run with: {@code ./gradlew deploy -Probot=HubberdShooterTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>LEFT BUMPER (hold) - Test Motor 1 ONLY (positive voltage)</li>
 *   <li>RIGHT BUMPER (hold) - Test Motor 2 ONLY (positive voltage)</li>
 *   <li>A (hold) - Intake mode (both motors pull fuel in)</li>
 *   <li>B (hold) - Outtake mode (both motors push fuel out)</li>
 *   <li>X (hold) - Shoot mode (motors counter-rotate)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 * <p>
 * <b>Configuration (via Elastic/Shuffleboard):</b>
 * <ul>
 *   <li>{@code HubberdShooter/TestPower%} - Power for individual motor tests (default 5%)</li>
 *   <li>{@code HubberdShooter/IntakePower%} - Power for intake mode (default 5%)</li>
 *   <li>{@code HubberdShooter/OuttakePower%} - Power for outtake mode (default 5%)</li>
 *   <li>{@code HubberdShooter/ShootingPower%} - Power for shooting mode (default 10%)</li>
 *   <li>{@code HubberdShooter/Motor1Inverted?} - Invert motor 1 direction</li>
 *   <li>{@code HubberdShooter/Motor2Inverted?} - Invert motor 2 direction</li>
 * </ul>
 * <p>
 * <b>Motor direction test procedure:</b>
 * <ol>
 *   <li>Hold LEFT BUMPER - observe which way Motor 1 spins</li>
 *   <li>Hold RIGHT BUMPER - observe which way Motor 2 spins</li>
 *   <li>If wrong direction, toggle Motor1Inverted? or Motor2Inverted?</li>
 * </ol>
 */
public class HubberdShooterTestbot extends TimedRobot {

    private HubberdShooterSubsystem shooter;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> HubberdShooterTestbot starting...");

        // Use appropriate hardware based on environment
        shooter = new HubberdShooterSubsystem(
                isSimulation() ? new HubberdShooterHardwareSim() : new HubberdShooterHardwareCTRE());
        controller = new GameController(0);

        // Set default command to idle
        shooter.setDefaultCommand(shooter.idleCommand());

        // Button bindings - Individual motor tests
        controller.leftBumper().whileTrue(shooter.testMotor1Command());
        controller.rightBumper().whileTrue(shooter.testMotor2Command());

        // Button bindings - Normal operation
        controller.a().whileTrue(shooter.intakeCommand());
        controller.b().whileTrue(shooter.outtakeCommand());
        controller.x().whileTrue(shooter.shootCommand());
        controller.y().onTrue(shooter.stopCommand());

        System.out.println(">>> Button mappings:");
        System.out.println("    LEFT BUMPER (hold) = Test Motor 1 ONLY (positive voltage)");
        System.out.println("    RIGHT BUMPER (hold) = Test Motor 2 ONLY (positive voltage)");
        System.out.println("    A (hold) = Intake (both motors pull fuel in)");
        System.out.println("    B (hold) = Outtake (both motors push fuel out)");
        System.out.println("    X (hold) = Shoot (motors counter-rotate)");
        System.out.println("    Y (press) = Stop all motors");
        System.out.println("");
        System.out.println(">>> Configuration:");
        System.out.println("    HubberdShooter/TestPower% = Power for individual motor tests (default 5%)");
        System.out.println("    HubberdShooter/IntakePower% = Power for intake mode");
        System.out.println("    HubberdShooter/OuttakePower% = Power for outtake mode");
        System.out.println("    HubberdShooter/ShootingPower% = Power for shooting mode");
        System.out.println("    HubberdShooter/Motor1Inverted? = Flip motor 1 direction");
        System.out.println("    HubberdShooter/Motor2Inverted? = Flip motor 2 direction");
        System.out.println("");
        System.out.println(">>> MOTOR DIRECTION TEST:");
        System.out.println("    1. Hold LEFT BUMPER - observe which way Motor 1 spins");
        System.out.println("    2. Hold RIGHT BUMPER - observe which way Motor 2 spins");
        System.out.println("    3. If wrong direction, toggle Motor1Inverted? or Motor2Inverted?");
        System.out.println("");
        System.out.println(">>> Expected behavior after correct inversion:");
        System.out.println("    Intake (A): Both motors positive RPM (pull in)");
        System.out.println("    Outtake (B): Both motors negative RPM (push out)");
        System.out.println("    Shoot (X): Motor1 positive, Motor2 negative (counter-rotate)");
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

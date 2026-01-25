package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intakefront.IntakeFrontHardwareSim;
import frc.robot.subsystems.intakefront.IntakeFrontHardwareSparkMax;
import frc.robot.subsystems.intakefront.IntakeFrontSubsystem;

/**
 * Standalone test program for the front intake subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=IntakeFrontTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Run intake (suck in)</li>
 *   <li>B (hold) - Run eject (spit out)</li>
 * </ul>
 * <p>
 * <b>Configuration (via Elastic/Shuffleboard):</b>
 * <ul>
 *   <li>{@code IntakeFront/SpeedPercent} - Motor speed (0-100%)</li>
 *   <li>{@code IntakeFront/Inverted?} - Toggle if motor spins wrong direction</li>
 * </ul>
 */
public class IntakeFrontTestbot extends TimedRobot {

    private IntakeFrontSubsystem intake;
    private CommandXboxController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> IntakeFrontTestbot starting...");

        // Use appropriate hardware based on environment
        intake = new IntakeFrontSubsystem(
                isSimulation() ? new IntakeFrontHardwareSim() : new IntakeFrontHardwareSparkMax());
        controller = new CommandXboxController(0);

        // Set default command to idle
        intake.setDefaultCommand(intake.idleCommand());

        // Button bindings
        controller.a().whileTrue(intake.intakeCommand());
        controller.b().whileTrue(intake.ejectCommand());

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (suck in)");
        System.out.println("    B (hold) = Eject (spit out)");
        System.out.println("");
        System.out.println(">>> Configuration:");
        System.out.println("    IntakeFront/SpeedPercent = Motor speed (0-100%)");
        System.out.println("    IntakeFront/Inverted? = Toggle if motor spins wrong direction");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}
}

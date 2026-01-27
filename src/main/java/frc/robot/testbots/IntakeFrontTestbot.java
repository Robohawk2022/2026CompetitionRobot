package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
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
 *   <li>{@code IntakeFront/IntakeSpeedRPS} - Intake speed in rev/sec</li>
 *   <li>{@code IntakeFront/EjectSpeedRPS} - Eject speed in rev/sec</li>
 *   <li>{@code IntakeFront/Inverted?} - Toggle if motor spins wrong direction</li>
 *   <li>{@code IntakeFront/kV} - Velocity feedforward gain</li>
 *   <li>{@code IntakeFront/kP} - Proportional gain</li>
 *   <li>{@code IntakeFront/StallThresholdRPM} - Stall detection velocity threshold</li>
 *   <li>{@code IntakeFront/StallTimeSec} - Stall detection time threshold</li>
 *   <li>{@code IntakeFront/CurrentLimit} - Motor current limit in amps</li>
 * </ul>
 * <p>
 * <b>Stall Detection:</b>
 * <p>
 * The intake monitors for stalls (low velocity while commanding motor).
 * When stalled for the configured time, the dashboard shows "Stalled? = true".
 * This indicates the hopper is likely full. In production, wire the stall
 * callback to an LED subsystem for driver alerts.
 */
public class IntakeFrontTestbot extends TimedRobot {

    private IntakeFrontSubsystem intake;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> IntakeFrontTestbot starting...");

        // Use appropriate hardware based on environment
        intake = new IntakeFrontSubsystem(
                isSimulation() ? new IntakeFrontHardwareSim() : new IntakeFrontHardwareSparkMax());
        controller = new GameController(0);

        // Set up stall callback (would wire to LED in production)
        intake.setStallCallback(stalled -> {
            if (stalled) {
                System.out.println(">>> STALL DETECTED - Hopper may be full!");
            } else {
                System.out.println(">>> Stall cleared");
            }
        });

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
        System.out.println("    IntakeFront/IntakeSpeedRPS = Intake speed (rev/sec)");
        System.out.println("    IntakeFront/EjectSpeedRPS = Eject speed (rev/sec)");
        System.out.println("    IntakeFront/Inverted? = Toggle if motor spins wrong direction");
        System.out.println("    IntakeFront/kV = Velocity feedforward gain");
        System.out.println("    IntakeFront/kP = Proportional gain");
        System.out.println("    IntakeFront/StallThresholdRPM = Stall detection threshold");
        System.out.println("    IntakeFront/StallTimeSec = Time before stall triggers");
        System.out.println("");
        System.out.println(">>> Stall Detection:");
        System.out.println("    Watch 'Stalled?' on dashboard - true indicates hopper full");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Stop motors immediately when robot is disabled
        intake.stop();
    }
}

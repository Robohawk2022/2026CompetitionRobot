package frc.robot.subsystems.led;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling LEDs via the REV Blinkin controller.
 * <p>
 * Uses {@link LEDSignal} to map robot states (like "intake full") to
 * LED colors/patterns. This provides a clean abstraction between robot
 * logic and LED display.
 * <p>
 * Example usage:
 * <pre>
 * // In RobotContainer, wire intake stall to LED signal
 * intake.setStallCallback(stalled -> {
 *     if (stalled) {
 *         led.setSignal(LEDSignal.INTAKE_FULL);
 *     } else {
 *         led.setSignal(LEDSignal.IDLE);
 *     }
 * });
 * </pre>
 */
public class LEDSubsystem extends SubsystemBase {

    //region Implementation --------------------------------------------------------

    private final LEDHardware hardware;
    private LEDSignal currentSignal = LEDSignal.OFF;

    /**
     * Creates a {@link LEDSubsystem}.
     *
     * @param hardware the LED hardware interface (required)
     */
    public LEDSubsystem(LEDHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);

        // Dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Signal", () -> currentSignal.name(), null);
            builder.addDoubleProperty("BlinkinCode", () -> currentSignal.getBlinkinCode(), null);
        });
    }

    @Override
    public void periodic() {
        // LED state is maintained by the Blinkin, no periodic updates needed
    }

    /**
     * @return the currently displayed signal
     */
    public LEDSignal getCurrentSignal() {
        return currentSignal;
    }

    //endregion

    //region Command factories -----------------------------------------------------

    /**
     * @param signal the signal to display
     * @return a command that sets the LED signal (runs once)
     */
    public Command signalCommand(LEDSignal signal) {
        return runOnce(() -> setSignal(signal))
            .withName("LED:" + signal.name());
    }

    /**
     * @param signal the signal to display while held
     * @return a command that displays a signal while running, then turns off
     */
    public Command signalWhileCommand(LEDSignal signal) {
        return startEnd(
            () -> setSignal(signal),
            () -> setSignal(LEDSignal.OFF)
        ).withName("LEDWhile:" + signal.name());
    }

    /**
     * @return a command that turns off the LEDs
     */
    public Command offCommand() {
        return runOnce(() -> setSignal(LEDSignal.OFF))
            .withName("LED:OFF");
    }

    /**
     * @return a command that does nothing (maintains current state)
     */
    public Command idleCommand() {
        return run(() -> {
            // Keep current state, do nothing
        }).withName("LED:Idle");
    }

    //endregion

    //region Direct control methods ------------------------------------------------

    /**
     * Directly sets the LED signal (for use outside command system).
     * <p>
     * Prefer using {@link #signalCommand(LEDSignal)} when possible.
     *
     * @param signal the signal to display
     */
    public void setSignal(LEDSignal signal) {
        if (signal == null) {
            signal = LEDSignal.OFF;
        }
        currentSignal = signal;
        hardware.applySignal(signal);
    }

    /**
     * Directly turns off the LEDs.
     */
    public void off() {
        setSignal(LEDSignal.OFF);
    }

    //endregion
}

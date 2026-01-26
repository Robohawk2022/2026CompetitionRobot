package frc.robot.subsystems.led;

/**
 * Interface for LED hardware.
 * <p>
 * Abstracts the LED hardware to allow different implementations:
 * <ul>
 *   <li>{@link LEDHardwareBlinkin} - REV Blinkin LED controller</li>
 *   <li>{@link LEDHardwareSim} - Simulation (posts signal name to dashboard)</li>
 * </ul>
 */
public interface LEDHardware {

    /**
     * Applies an LED signal (color/pattern) to the hardware.
     *
     * @param signal the signal to display
     */
    void applySignal(LEDSignal signal);

    /**
     * @return the currently applied signal, or null if none
     */
    LEDSignal getCurrentSignal();

    /**
     * Turns off the LEDs.
     */
    default void off() {
        applySignal(LEDSignal.OFF);
    }
}

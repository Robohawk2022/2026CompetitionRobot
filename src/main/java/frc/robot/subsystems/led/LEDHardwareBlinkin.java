package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import static frc.robot.Config.LED.*;

/**
 * Implements {@link LEDHardware} using the REV Blinkin LED controller.
 * <p>
 * The Blinkin is controlled like a Spark motor controller via PWM.
 * Different "speeds" (-1.0 to 1.0) select different colors and patterns.
 *
 * @see <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf">Blinkin User Manual</a>
 */
public class LEDHardwareBlinkin implements LEDHardware {

    private final Spark blinkin;
    private LEDSignal currentSignal = LEDSignal.OFF;

    /**
     * Creates the Blinkin hardware on the configured PWM port.
     */
    public LEDHardwareBlinkin() {
        blinkin = new Spark(pwmPort);
    }

    @Override
    public void applySignal(LEDSignal signal) {
        if (signal == null) {
            signal = LEDSignal.OFF;
        }
        currentSignal = signal;
        blinkin.set(signal.getBlinkinCode());
    }

    @Override
    public LEDSignal getCurrentSignal() {
        return currentSignal;
    }
}

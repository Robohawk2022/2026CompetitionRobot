package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Config.LED.*;

/**
 * Implements {@link LEDHardware} using WPILib's AddressableLED.
 * <p>
 * Works with WS2812/NeoPixel style LED strips connected to a PWM port.
 */
public class LEDHardwareWPILib implements LEDHardware {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    /**
     * Creates the LED hardware on the configured PWM port.
     */
    public LEDHardwareWPILib() {
        led = new AddressableLED(PWM_PORT);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void setSolidColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    @Override
    public void setLED(int index, Color color) {
        if (index >= 0 && index < buffer.getLength()) {
            buffer.setLED(index, color);
        }
    }

    @Override
    public void update() {
        led.setData(buffer);
    }

    @Override
    public int getLength() {
        return buffer.getLength();
    }

    @Override
    public void off() {
        setSolidColor(Color.kBlack);
        update();
    }
}

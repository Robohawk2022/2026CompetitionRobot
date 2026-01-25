package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Interface for LED strip hardware.
 * <p>
 * Abstracts the LED hardware to allow simulation and testing.
 */
public interface LEDHardware {

    /**
     * Sets all LEDs to a solid color.
     *
     * @param color the color to set
     */
    void setSolidColor(Color color);

    /**
     * Sets a specific LED to a color.
     *
     * @param index the LED index (0-based)
     * @param color the color to set
     */
    void setLED(int index, Color color);

    /**
     * Updates the LED strip with the current buffer.
     * Call this after making changes to apply them.
     */
    void update();

    /**
     * @return the number of LEDs in the strip
     */
    int getLength();

    /**
     * Turns off all LEDs.
     */
    void off();
}

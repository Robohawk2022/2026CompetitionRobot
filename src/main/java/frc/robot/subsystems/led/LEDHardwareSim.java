package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Config.LED.*;

/**
 * Implements {@link LEDHardware} for simulation.
 * <p>
 * Stores LED colors in memory for dashboard visualization.
 */
public class LEDHardwareSim implements LEDHardware {

    private final Color[] leds;
    private Color currentSolidColor = Color.kBlack;

    /**
     * Creates the simulated LED hardware.
     */
    public LEDHardwareSim() {
        leds = new Color[LED_COUNT];
        for (int i = 0; i < leds.length; i++) {
            leds[i] = Color.kBlack;
        }
    }

    @Override
    public void setSolidColor(Color color) {
        currentSolidColor = color;
        for (int i = 0; i < leds.length; i++) {
            leds[i] = color;
        }
    }

    @Override
    public void setLED(int index, Color color) {
        if (index >= 0 && index < leds.length) {
            leds[index] = color;
        }
    }

    @Override
    public void update() {
        // In simulation, updates are immediate (no hardware to sync)
    }

    @Override
    public int getLength() {
        return leds.length;
    }

    @Override
    public void off() {
        setSolidColor(Color.kBlack);
    }

    /**
     * @return the current solid color (for dashboard display)
     */
    public Color getCurrentColor() {
        return currentSolidColor;
    }

    /**
     * @param index the LED index
     * @return the color at that index
     */
    public Color getLED(int index) {
        if (index >= 0 && index < leds.length) {
            return leds[index];
        }
        return Color.kBlack;
    }
}

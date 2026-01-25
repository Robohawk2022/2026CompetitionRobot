package frc.robot.subsystems.led;

import java.util.Objects;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Config.LED.*;

/**
 * Subsystem for controlling LED light strips on the robot.
 * <p>
 * Supports multiple patterns including solid colors, blinking, rainbow, and more.
 * Colors and patterns can be changed dynamically via commands.
 */
public class LEDSubsystem extends SubsystemBase {

    //region Preset Colors ----------------------------------------------------------

    /**
     * Preset colors for common robot states.
     */
    public enum LEDColor {
        OFF(Color.kBlack),
        RED(Color.kRed),
        GREEN(Color.kGreen),
        BLUE(Color.kBlue),
        YELLOW(Color.kYellow),
        ORANGE(Color.kOrange),
        PURPLE(Color.kPurple),
        CYAN(Color.kCyan),
        WHITE(Color.kWhite),
        PINK(Color.kHotPink),
        ALLIANCE_RED(Color.kFirstRed),
        ALLIANCE_BLUE(Color.kFirstBlue);

        private final Color color;

        LEDColor(Color color) {
            this.color = color;
        }

        public Color getColor() {
            return color;
        }
    }

    /**
     * Available LED patterns.
     */
    public enum LEDPattern {
        SOLID,
        BLINK,
        RAINBOW,
        BREATHING,
        CHASE
    }

    //endregion

    //region Implementation --------------------------------------------------------

    private final LEDHardware hardware;
    private String currentMode = "off";
    private Color currentColor = Color.kBlack;
    private LEDPattern currentPattern = LEDPattern.SOLID;

    // Pattern state
    private int rainbowOffset = 0;
    private double breathingPhase = 0;
    private int chaseOffset = 0;

    /**
     * Creates a {@link LEDSubsystem}.
     *
     * @param hardware the LED hardware interface (required)
     */
    public LEDSubsystem(LEDHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);

        // Dashboard telemetry for Elastic visualization
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addStringProperty("Pattern", () -> currentPattern.name(), null);
            builder.addStringProperty("Color", this::getColorName, null);
            builder.addDoubleProperty("ColorR", () -> currentColor.red * 255, null);
            builder.addDoubleProperty("ColorG", () -> currentColor.green * 255, null);
            builder.addDoubleProperty("ColorB", () -> currentColor.blue * 255, null);
            // Hex color for easy copying
            builder.addStringProperty("ColorHex", this::getColorHex, null);
        });
    }

    @Override
    public void periodic() {
        // Patterns that need continuous updates are handled in their commands
    }

    /**
     * @return the current color as a hex string (e.g., "#FF0000")
     */
    private String getColorHex() {
        int r = (int) (currentColor.red * 255);
        int g = (int) (currentColor.green * 255);
        int b = (int) (currentColor.blue * 255);
        return String.format("#%02X%02X%02X", r, g, b);
    }

    /**
     * @return the name of the current color if it matches a preset
     */
    private String getColorName() {
        for (LEDColor preset : LEDColor.values()) {
            if (colorsMatch(currentColor, preset.getColor())) {
                return preset.name();
            }
        }
        return getColorHex();
    }

    private boolean colorsMatch(Color a, Color b) {
        return Math.abs(a.red - b.red) < 0.01
            && Math.abs(a.green - b.green) < 0.01
            && Math.abs(a.blue - b.blue) < 0.01;
    }

    /**
     * Applies brightness scaling to a color.
     */
    private Color applyBrightness(Color color) {
        double b = brightness.getAsDouble();
        return new Color(color.red * b, color.green * b, color.blue * b);
    }

    //endregion

    //region Command factories -----------------------------------------------------

    /**
     * @return a command that turns off all LEDs
     */
    public Command offCommand() {
        return runOnce(() -> {
            currentMode = "off";
            currentColor = Color.kBlack;
            currentPattern = LEDPattern.SOLID;
            hardware.off();
        });
    }

    /**
     * @param color the color preset
     * @return a command that sets a solid color
     */
    public Command solidCommand(LEDColor color) {
        return solidCommand(color.getColor());
    }

    /**
     * @param color the color to set
     * @return a command that sets a solid color
     */
    public Command solidCommand(Color color) {
        return run(() -> {
            currentMode = "solid";
            currentColor = color;
            currentPattern = LEDPattern.SOLID;
            hardware.setSolidColor(applyBrightness(color));
            hardware.update();
        });
    }

    /**
     * @param color the color preset to blink
     * @return a command that blinks the LEDs
     */
    public Command blinkCommand(LEDColor color) {
        return blinkCommand(color.getColor());
    }

    /**
     * @param color the color to blink
     * @return a command that blinks the LEDs on and off
     */
    public Command blinkCommand(Color color) {
        return run(() -> {
            currentMode = "blink";
            currentColor = color;
            currentPattern = LEDPattern.BLINK;

            double period = blinkPeriod.getAsDouble();
            boolean on = (Timer.getFPGATimestamp() % period) < (period / 2);

            if (on) {
                hardware.setSolidColor(applyBrightness(color));
            } else {
                hardware.setSolidColor(Color.kBlack);
            }
            hardware.update();
        });
    }

    /**
     * @return a command that displays a rainbow pattern
     */
    public Command rainbowCommand() {
        return run(() -> {
            currentMode = "rainbow";
            currentPattern = LEDPattern.RAINBOW;

            int length = hardware.getLength();
            double speed = rainbowSpeed.getAsDouble();

            for (int i = 0; i < length; i++) {
                // Calculate hue for this LED (0-180 for WPILib HSV)
                int hue = (rainbowOffset + (i * 180 / length)) % 180;
                Color color = Color.fromHSV(hue, 255, (int) (255 * brightness.getAsDouble()));
                hardware.setLED(i, color);

                // Update currentColor to first LED color for dashboard
                if (i == 0) {
                    currentColor = color;
                }
            }

            rainbowOffset += (int) speed;
            rainbowOffset %= 180;

            hardware.update();
        });
    }

    /**
     * @param color the color preset to breathe
     * @return a command that creates a breathing/pulsing effect
     */
    public Command breathingCommand(LEDColor color) {
        return breathingCommand(color.getColor());
    }

    /**
     * @param color the color to breathe
     * @return a command that creates a breathing/pulsing effect
     */
    public Command breathingCommand(Color color) {
        return run(() -> {
            currentMode = "breathing";
            currentColor = color;
            currentPattern = LEDPattern.BREATHING;

            // Sinusoidal brightness modulation
            double period = blinkPeriod.getAsDouble() * 2; // Full cycle
            breathingPhase = (Timer.getFPGATimestamp() % period) / period * 2 * Math.PI;
            double factor = (Math.sin(breathingPhase) + 1) / 2; // 0 to 1
            factor *= brightness.getAsDouble();

            Color dimmed = new Color(color.red * factor, color.green * factor, color.blue * factor);
            hardware.setSolidColor(dimmed);
            hardware.update();
        });
    }

    /**
     * @param color the color preset for the chase
     * @return a command that creates a chase/running light effect
     */
    public Command chaseCommand(LEDColor color) {
        return chaseCommand(color.getColor());
    }

    /**
     * @param color the color for the chase
     * @return a command that creates a chase/running light effect
     */
    public Command chaseCommand(Color color) {
        return run(() -> {
            currentMode = "chase";
            currentColor = color;
            currentPattern = LEDPattern.CHASE;

            int length = hardware.getLength();
            int chaseLength = Math.max(3, length / 10); // 10% of strip or at least 3

            for (int i = 0; i < length; i++) {
                int distance = Math.abs(i - chaseOffset);
                if (distance > length / 2) {
                    distance = length - distance; // Wrap around
                }

                if (distance < chaseLength) {
                    double factor = 1.0 - ((double) distance / chaseLength);
                    factor *= brightness.getAsDouble();
                    Color dimmed = new Color(color.red * factor, color.green * factor, color.blue * factor);
                    hardware.setLED(i, dimmed);
                } else {
                    hardware.setLED(i, Color.kBlack);
                }
            }

            chaseOffset = (chaseOffset + 1) % length;
            hardware.update();
        });
    }

    /**
     * Sets LEDs based on a percentage (e.g., for battery level, scoring progress).
     *
     * @param color   the color for filled portion
     * @param percent the percentage to fill (0.0 to 1.0)
     * @return a command that displays a progress bar
     */
    public Command progressCommand(LEDColor color, double percent) {
        return progressCommand(color.getColor(), percent);
    }

    /**
     * Sets LEDs based on a percentage (e.g., for battery level, scoring progress).
     *
     * @param color   the color for filled portion
     * @param percent the percentage to fill (0.0 to 1.0)
     * @return a command that displays a progress bar
     */
    public Command progressCommand(Color color, double percent) {
        return run(() -> {
            currentMode = "progress";
            currentColor = color;
            currentPattern = LEDPattern.SOLID;

            int length = hardware.getLength();
            int filledCount = (int) (length * Math.max(0, Math.min(1, percent)));

            for (int i = 0; i < length; i++) {
                if (i < filledCount) {
                    hardware.setLED(i, applyBrightness(color));
                } else {
                    hardware.setLED(i, Color.kBlack);
                }
            }
            hardware.update();
        });
    }

    /**
     * @return a command that does nothing (idle state)
     */
    public Command idleCommand() {
        return run(() -> {
            // Keep current state, do nothing
        });
    }

    //endregion

    //region Direct control methods ------------------------------------------------

    /**
     * Directly sets a solid color (for use outside command system).
     *
     * @param color the color to set
     */
    public void setSolid(LEDColor color) {
        setSolid(color.getColor());
    }

    /**
     * Directly sets a solid color (for use outside command system).
     *
     * @param color the color to set
     */
    public void setSolid(Color color) {
        currentMode = "solid";
        currentColor = color;
        currentPattern = LEDPattern.SOLID;
        hardware.setSolidColor(applyBrightness(color));
        hardware.update();
    }

    /**
     * Directly sets blinking pattern (call repeatedly for animation).
     *
     * @param color the color to blink
     */
    public void setBlink(LEDColor color) {
        setBlink(color.getColor());
    }

    /**
     * Directly sets blinking pattern (call repeatedly for animation).
     *
     * @param color the color to blink
     */
    public void setBlink(Color color) {
        currentMode = "blink";
        currentColor = color;
        currentPattern = LEDPattern.BLINK;

        double period = blinkPeriod.getAsDouble();
        boolean on = (Timer.getFPGATimestamp() % period) < (period / 2);

        if (on) {
            hardware.setSolidColor(applyBrightness(color));
        } else {
            hardware.setSolidColor(Color.kBlack);
        }
        hardware.update();
    }

    /**
     * Directly updates rainbow pattern (call repeatedly for animation).
     */
    public void setRainbow() {
        currentMode = "rainbow";
        currentPattern = LEDPattern.RAINBOW;

        int length = hardware.getLength();
        double speed = rainbowSpeed.getAsDouble();

        for (int i = 0; i < length; i++) {
            int hue = (rainbowOffset + (i * 180 / length)) % 180;
            Color color = Color.fromHSV(hue, 255, (int) (255 * brightness.getAsDouble()));
            hardware.setLED(i, color);

            if (i == 0) {
                currentColor = color;
            }
        }

        rainbowOffset += (int) speed;
        rainbowOffset %= 180;

        hardware.update();
    }

    /**
     * Directly sets breathing pattern (call repeatedly for animation).
     *
     * @param color the color to breathe
     */
    public void setBreathing(LEDColor color) {
        setBreathing(color.getColor());
    }

    /**
     * Directly sets breathing pattern (call repeatedly for animation).
     *
     * @param color the color to breathe
     */
    public void setBreathing(Color color) {
        currentMode = "breathing";
        currentColor = color;
        currentPattern = LEDPattern.BREATHING;

        double period = blinkPeriod.getAsDouble() * 2;
        breathingPhase = (Timer.getFPGATimestamp() % period) / period * 2 * Math.PI;
        double factor = (Math.sin(breathingPhase) + 1) / 2;
        factor *= brightness.getAsDouble();

        Color dimmed = new Color(color.red * factor, color.green * factor, color.blue * factor);
        hardware.setSolidColor(dimmed);
        hardware.update();
    }

    /**
     * Directly sets chase pattern (call repeatedly for animation).
     *
     * @param color the color for the chase
     */
    public void setChase(LEDColor color) {
        setChase(color.getColor());
    }

    /**
     * Directly sets chase pattern (call repeatedly for animation).
     *
     * @param color the color for the chase
     */
    public void setChase(Color color) {
        currentMode = "chase";
        currentColor = color;
        currentPattern = LEDPattern.CHASE;

        int length = hardware.getLength();
        int chaseLength = Math.max(3, length / 10);

        for (int i = 0; i < length; i++) {
            int distance = Math.abs(i - chaseOffset);
            if (distance > length / 2) {
                distance = length - distance;
            }

            if (distance < chaseLength) {
                double factor = 1.0 - ((double) distance / chaseLength);
                factor *= brightness.getAsDouble();
                Color dimmed = new Color(color.red * factor, color.green * factor, color.blue * factor);
                hardware.setLED(i, dimmed);
            } else {
                hardware.setLED(i, Color.kBlack);
            }
        }

        chaseOffset = (chaseOffset + 1) % length;
        hardware.update();
    }

    /**
     * Directly turns off all LEDs.
     */
    public void setOff() {
        currentMode = "off";
        currentColor = Color.kBlack;
        currentPattern = LEDPattern.SOLID;
        hardware.off();
    }

    //endregion
}

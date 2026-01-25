package frc.robot.subsystems.led;

/**
 * LED signals that map robot states to REV Blinkin pattern codes.
 * <p>
 * Each signal represents a meaningful robot state (e.g., "intake is full")
 * and maps to a specific Blinkin color/pattern code. This provides a clean
 * abstraction between robot logic and LED display.
 * <p>
 * Blinkin codes range from -1.0 to 1.0:
 * <ul>
 *   <li>-0.99 to -0.01: Fixed palette patterns (rainbow, fire, etc.)</li>
 *   <li>0.01 to 0.99: Solid colors and color patterns</li>
 * </ul>
 *
 * @see <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf">Blinkin User Manual</a>
 */
public enum LEDSignal {

    //=======================================================================
    // Robot state signals - map meanings to patterns
    //=======================================================================

    /** Robot is disabled/idle */
    DISABLED(BlinkinCode.BREATH_GRAY),

    /** Robot is enabled but nothing active */
    IDLE(BlinkinCode.SOLID_BLUE),

    /** Intake is running (actively collecting) */
    INTAKING(BlinkinCode.SOLID_GREEN),

    /** Intake is full / hopper full (stall detected) */
    INTAKE_FULL(BlinkinCode.STROBE_RED),

    /** Ejecting game pieces */
    EJECTING(BlinkinCode.SOLID_ORANGE),

    /** Robot is in shooting range - close distance */
    SHOOT_RANGE_CLOSE(BlinkinCode.SOLID_GREEN),

    /** Robot is in shooting range - medium distance */
    SHOOT_RANGE_MEDIUM(BlinkinCode.SOLID_YELLOW),

    /** Robot is in shooting range - far distance */
    SHOOT_RANGE_FAR(BlinkinCode.SOLID_RED),

    /** Shooter at target speed, ready to fire */
    READY_TO_SHOOT(BlinkinCode.STROBE_GOLD),

    /** Vision target acquired */
    TARGET_LOCKED(BlinkinCode.SOLID_LIME),

    /** Vision searching for target */
    TARGET_SEARCHING(BlinkinCode.BREATH_BLUE),

    /** Autonomous mode active */
    AUTO_MODE(BlinkinCode.RAINBOW_RAINBOW),

    /** Endgame warning (last 20 seconds) */
    ENDGAME_WARNING(BlinkinCode.STROBE_BLUE),

    /** Climbing sequence active */
    CLIMBING(BlinkinCode.COLOR_WAVES_PARTY),

    /** Error / fault condition */
    ERROR(BlinkinCode.STROBE_RED),

    /** Alliance color - Red */
    ALLIANCE_RED(BlinkinCode.SOLID_RED),

    /** Alliance color - Blue */
    ALLIANCE_BLUE(BlinkinCode.SOLID_BLUE),

    /** Celebration / scoring confirmed */
    CELEBRATION(BlinkinCode.RAINBOW_PARTY),

    /** All LEDs off */
    OFF(BlinkinCode.SOLID_BLACK);

    //=======================================================================
    // Implementation
    //=======================================================================

    private final double blinkinCode;

    LEDSignal(double blinkinCode) {
        this.blinkinCode = blinkinCode;
    }

    /**
     * @return the REV Blinkin code for this signal (-1.0 to 1.0)
     */
    public double getBlinkinCode() {
        return blinkinCode;
    }

    //=======================================================================
    // REV Blinkin color/pattern codes
    // Reference: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
    //=======================================================================

    /**
     * Constants for REV Blinkin LED controller codes.
     * <p>
     * These are the raw double values sent to the Blinkin via PWM.
     */
    public interface BlinkinCode {

        // Fixed palette patterns
        double RAINBOW_RAINBOW = -0.99;
        double RAINBOW_PARTY = -0.97;
        double RAINBOW_OCEAN = -0.95;
        double RAINBOW_LAVA = -0.93;
        double RAINBOW_FOREST = -0.91;
        double RAINBOW_GLITTER = -0.89;
        double CONFETTI = -0.87;
        double SHOT_RED = -0.85;
        double SHOT_BLUE = -0.83;
        double SHOT_WHITE = -0.81;
        double SINELON_RAINBOW = -0.79;
        double SINELON_PARTY = -0.77;
        double SINELON_OCEAN = -0.75;
        double SINELON_LAVA = -0.73;
        double SINELON_FOREST = -0.71;
        double BPM_RAINBOW = -0.69;
        double BPM_PARTY = -0.67;
        double BPM_OCEAN = -0.65;
        double BPM_LAVA = -0.63;
        double BPM_FOREST = -0.61;
        double FIRE_MEDIUM = -0.59;
        double FIRE_LARGE = -0.57;
        double TWINKLES_RAINBOW = -0.55;
        double TWINKLES_PARTY = -0.53;
        double TWINKLES_OCEAN = -0.51;
        double TWINKLES_LAVA = -0.49;
        double TWINKLES_FOREST = -0.47;
        double COLOR_WAVES_RAINBOW = -0.45;
        double COLOR_WAVES_PARTY = -0.43;
        double COLOR_WAVES_OCEAN = -0.41;
        double COLOR_WAVES_LAVA = -0.39;
        double COLOR_WAVES_FOREST = -0.37;
        double LARSON_SCANNER_RED = -0.35;
        double LARSON_SCANNER_GRAY = -0.33;
        double LIGHT_CHASE_RED = -0.31;
        double LIGHT_CHASE_BLUE = -0.29;
        double LIGHT_CHASE_GRAY = -0.27;
        double HEARTBEAT_RED = -0.25;
        double HEARTBEAT_BLUE = -0.23;
        double HEARTBEAT_WHITE = -0.21;
        double HEARTBEAT_GRAY = -0.19;
        double BREATH_RED = -0.17;
        double BREATH_BLUE = -0.15;
        double BREATH_GRAY = -0.13;
        double STROBE_RED = -0.11;
        double STROBE_BLUE = -0.09;
        double STROBE_GOLD = -0.07;
        double STROBE_WHITE = -0.05;

        // Color 1 patterns (use setColor1 to change)
        double END_BLEND_BLACK_1 = -0.03;
        double LARSON_SCANNER_1 = -0.01;
        double LIGHT_CHASE_1 = 0.01;
        double HEARTBEAT_SLOW_1 = 0.03;
        double HEARTBEAT_MEDIUM_1 = 0.05;
        double HEARTBEAT_FAST_1 = 0.07;
        double BREATH_SLOW_1 = 0.09;
        double BREATH_FAST_1 = 0.11;
        double SHOT_1 = 0.13;
        double STROBE_1 = 0.15;

        // Color 2 patterns
        double END_BLEND_BLACK_2 = 0.17;
        double LARSON_SCANNER_2 = 0.19;
        double LIGHT_CHASE_2 = 0.21;
        double HEARTBEAT_SLOW_2 = 0.23;
        double HEARTBEAT_MEDIUM_2 = 0.25;
        double HEARTBEAT_FAST_2 = 0.27;
        double BREATH_SLOW_2 = 0.29;
        double BREATH_FAST_2 = 0.31;
        double SHOT_2 = 0.33;
        double STROBE_2 = 0.35;

        // Color 1 and 2 patterns
        double SPARKLE_1_ON_2 = 0.37;
        double SPARKLE_2_ON_1 = 0.39;
        double GRADIENT_1_2 = 0.41;
        double BPM_1_2 = 0.43;
        double END_BLEND_1_2 = 0.45;
        double END_BLEND_2 = 0.47;
        double COLOR_1_2_NO_BLEND = 0.49;
        double TWINKLES_1_2 = 0.51;
        double COLOR_WAVES_1_2 = 0.53;
        double SINELON_1_2 = 0.55;

        // Solid colors
        double SOLID_HOT_PINK = 0.57;
        double SOLID_DARK_RED = 0.59;
        double SOLID_RED = 0.61;
        double SOLID_RED_ORANGE = 0.63;
        double SOLID_ORANGE = 0.65;
        double SOLID_GOLD = 0.67;
        double SOLID_YELLOW = 0.69;
        double SOLID_LAWN_GREEN = 0.71;
        double SOLID_LIME = 0.73;
        double SOLID_DARK_GREEN = 0.75;
        double SOLID_GREEN = 0.77;
        double SOLID_BLUE_GREEN = 0.79;
        double SOLID_AQUA = 0.81;
        double SOLID_SKY_BLUE = 0.83;
        double SOLID_DARK_BLUE = 0.85;
        double SOLID_BLUE = 0.87;
        double SOLID_BLUE_VIOLET = 0.89;
        double SOLID_VIOLET = 0.91;
        double SOLID_WHITE = 0.93;
        double SOLID_GRAY = 0.95;
        double SOLID_DARK_GRAY = 0.97;
        double SOLID_BLACK = 0.99;
    }
}

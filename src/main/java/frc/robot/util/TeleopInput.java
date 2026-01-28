package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.GameController;

import static frc.robot.Config.SwerveTeleop.*;

/**
 * Helper class for teleop input processing shared between swerve commands.
 * Provides mode detection, input conditioning, and speed factor calculations.
 */
public class TeleopInput {

    /** Speed mode based on trigger state */
    public enum TurboSniperMode {
        SNIPER,
        TURBO,
        NORMAL;

        /** @return multiplier for speeds in this mode */
        public double getSpeedFactor() {
            return switch (this) {
                case SNIPER -> sniperFactor.getAsDouble();
                case TURBO -> turboFactor.getAsDouble();
                case NORMAL -> 1.0;
            };
        }

        /** @return multiplier for rotation speed */
        public boolean applyFactorToRotation() {
            return applySniperToRotation.getAsBoolean();
        }
    }

    /**
     * Gets the current speed mode based on controller triggers.
     * @param controller the game controller
     * @return SNIPER if left trigger, TURBO if right trigger, else NORMAL
     */
    public static TurboSniperMode getMode(GameController controller) {
        if (controller.leftTriggerSupplier().getAsBoolean()) {
            return TurboSniperMode.SNIPER;
        }
        if (controller.rightTriggerSupplier().getAsBoolean()) {
            return TurboSniperMode.TURBO;
        }
        return TurboSniperMode.NORMAL;
    }

    /**
     * Applies deadband and exponent to joystick input.
     * @param input raw joystick value (-1.0 to 1.0)
     * @return conditioned input value
     */
    public static double conditionInput(double input) {
        double result = MathUtil.clamp(input, -1.0, 1.0);
        result = MathUtil.applyDeadband(result, deadband.getAsDouble());
        result = Math.copySign(Math.pow(Math.abs(result), exponent.getAsDouble()), input);
        return result;
    }
}

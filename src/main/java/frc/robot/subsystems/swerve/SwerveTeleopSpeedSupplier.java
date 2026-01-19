package frc.robot.subsystems.swerve;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Config.Swerve.*;

/**
 * Encapsulates teleop joystick input handling for swerve drive.
 * <p>
 * Handles:
 * <ul>
 *   <li>Left stick Y for forward/backward (inverted)</li>
 *   <li>Left stick X for strafe (inverted)</li>
 *   <li>Right stick X for rotation (inverted)</li>
 *   <li>Left trigger for sniper mode</li>
 *   <li>Right trigger for turbo mode</li>
 * </ul>
 * <p>
 * Applies deadband from {@code Config.Swerve.deadzone}.
 * <p>
 * Supports two controller types via {@code Config.Swerve.useXboxMapping}:
 * <ul>
 *   <li>8BitDo (default): Right stick X on axis 2, triggers on axes 4/5 (-1 to 1)</li>
 *   <li>Xbox (for simulation): Right stick X on axis 4, triggers on axes 2/3 (0 to 1)</li>
 * </ul>
 */
public class SwerveTeleopSpeedSupplier {

    // Axis indices for 8BitDo controller
    private static final int BITDO_LEFT_X_AXIS = 0;
    private static final int BITDO_LEFT_Y_AXIS = 1;
    private static final int BITDO_RIGHT_X_AXIS = 2;
    private static final int BITDO_LEFT_TRIGGER_AXIS = 4;
    private static final int BITDO_RIGHT_TRIGGER_AXIS = 5;

    // Axis indices for Xbox controller (WPILib standard)
    private static final int XBOX_LEFT_X_AXIS = 0;
    private static final int XBOX_LEFT_Y_AXIS = 1;
    private static final int XBOX_RIGHT_X_AXIS = 4;
    private static final int XBOX_LEFT_TRIGGER_AXIS = 2;
    private static final int XBOX_RIGHT_TRIGGER_AXIS = 3;

    private final CommandXboxController controller;

    /**
     * Creates a {@link SwerveTeleopSpeedSupplier}.
     *
     * @param controller the Xbox controller for driver input (required)
     */
    public SwerveTeleopSpeedSupplier(CommandXboxController controller) {
        this.controller = Objects.requireNonNull(controller);
    }

    /**
     * @return supplier for forward/backward speed (-1 to 1), with deadband applied
     */
    public DoubleSupplier xSupplier() {
        return () -> {
            int axis = useXboxMapping.getAsBoolean() ? XBOX_LEFT_Y_AXIS : BITDO_LEFT_Y_AXIS;
            return -MathUtil.applyDeadband(controller.getHID().getRawAxis(axis), deadzone.getAsDouble());
        };
    }

    /**
     * @return supplier for left/right strafe speed (-1 to 1), with deadband applied
     */
    public DoubleSupplier ySupplier() {
        return () -> {
            int axis = useXboxMapping.getAsBoolean() ? XBOX_LEFT_X_AXIS : BITDO_LEFT_X_AXIS;
            return -MathUtil.applyDeadband(controller.getHID().getRawAxis(axis), deadzone.getAsDouble());
        };
    }

    /**
     * @return supplier for rotation speed (-1 to 1), with deadband applied
     */
    public DoubleSupplier rotSupplier() {
        return () -> {
            int axis = useXboxMapping.getAsBoolean() ? XBOX_RIGHT_X_AXIS : BITDO_RIGHT_X_AXIS;
            return -MathUtil.applyDeadband(controller.getHID().getRawAxis(axis), deadzone.getAsDouble());
        };
    }

    /**
     * @return supplier for sniper mode trigger axis (0 to 1)
     */
    public DoubleSupplier sniperTrigger() {
        return () -> {
            if (useXboxMapping.getAsBoolean()) {
                // Xbox triggers: 0 (not pressed) to 1 (fully pressed)
                return controller.getHID().getRawAxis(XBOX_LEFT_TRIGGER_AXIS);
            } else {
                // 8BitDo triggers: -1 (not pressed) to 1 (fully pressed), normalize to 0-1
                return (controller.getHID().getRawAxis(BITDO_LEFT_TRIGGER_AXIS) + 1) / 2;
            }
        };
    }

    /**
     * @return supplier for turbo mode trigger axis (0 to 1)
     */
    public DoubleSupplier turboTrigger() {
        return () -> {
            if (useXboxMapping.getAsBoolean()) {
                // Xbox triggers: 0 (not pressed) to 1 (fully pressed)
                return controller.getHID().getRawAxis(XBOX_RIGHT_TRIGGER_AXIS);
            } else {
                // 8BitDo triggers: -1 (not pressed) to 1 (fully pressed), normalize to 0-1
                return (controller.getHID().getRawAxis(BITDO_RIGHT_TRIGGER_AXIS) + 1) / 2;
            }
        };
    }

    /**
     * Creates a teleop drive command with turbo and sniper modes.
     *
     * @param swerve the swerve subsystem to control
     * @return the drive command (field-relative)
     */
    public Command driveCommand(SwerveSubsystem swerve) {
        return driveCommand(swerve, true);
    }

    /**
     * Creates a teleop drive command with turbo and sniper modes.
     *
     * @param swerve        the swerve subsystem to control
     * @param fieldRelative whether to drive field-relative
     * @return the drive command
     */
    public Command driveCommand(SwerveSubsystem swerve, boolean fieldRelative) {
        return swerve.driveCommand(
                xSupplier(),
                ySupplier(),
                rotSupplier(),
                sniperTrigger(),
                turboTrigger(),
                fieldRelative);
    }
}

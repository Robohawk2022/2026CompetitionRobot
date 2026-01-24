package frc.robot.subsystems.swerve;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    // Axis indices for 8BitDo Ultimate controller (DirectInput mode)
    private static final int BITDO_LEFT_X_AXIS = 0;
    private static final int BITDO_LEFT_Y_AXIS = 1;
    private static final int BITDO_RIGHT_X_AXIS = 2;
    private static final int BITDO_LEFT_TRIGGER_AXIS = 5;   // swapped in DirectInput mode
    private static final int BITDO_RIGHT_TRIGGER_AXIS = 4;  // swapped in DirectInput mode

    // Button indices for 8BitDo Ultimate controller (DirectInput mode, WPILib 1-indexed)
    // Based on SDL GameControllerDB mappings for 8BitDo Ultimate
    private static final int BITDO_A_BUTTON = 1;            // b0 + 1
    private static final int BITDO_B_BUTTON = 2;            // b1 + 1
    private static final int BITDO_X_BUTTON = 4;            // b3 + 1
    private static final int BITDO_Y_BUTTON = 5;            // b4 + 1
    private static final int BITDO_LEFT_BUMPER = 7;         // b6 + 1
    private static final int BITDO_RIGHT_BUMPER = 8;        // b7 + 1
    private static final int BITDO_BACK_BUTTON = 11;        // b10 + 1
    private static final int BITDO_START_BUTTON = 12;       // b11 + 1
    private static final int BITDO_LEFT_STICK_BUTTON = 14;  // b13 + 1
    private static final int BITDO_RIGHT_STICK_BUTTON = 15; // b14 + 1

    // Axis indices for Xbox controller (WPILib standard)
    private static final int XBOX_LEFT_X_AXIS = 0;
    private static final int XBOX_LEFT_Y_AXIS = 1;
    private static final int XBOX_RIGHT_X_AXIS = 4;
    private static final int XBOX_LEFT_TRIGGER_AXIS = 2;
    private static final int XBOX_RIGHT_TRIGGER_AXIS = 3;

    // Button indices for Xbox controller (WPILib standard)
    private static final int XBOX_A_BUTTON = 1;
    private static final int XBOX_B_BUTTON = 2;
    private static final int XBOX_X_BUTTON = 3;
    private static final int XBOX_Y_BUTTON = 4;
    private static final int XBOX_LEFT_BUMPER = 5;
    private static final int XBOX_RIGHT_BUMPER = 6;
    private static final int XBOX_BACK_BUTTON = 7;
    private static final int XBOX_START_BUTTON = 8;
    private static final int XBOX_LEFT_STICK_BUTTON = 9;
    private static final int XBOX_RIGHT_STICK_BUTTON = 10;

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

    /**
     * Creates an orbit drive command using controller inputs.
     * <p>
     * Maps controls as follows:
     * <ul>
     *   <li>Left stick Y: Radial movement (toward/away from target)</li>
     *   <li>Left stick X: Tangential movement (orbit around target)</li>
     *   <li>Right stick X: Fine heading adjustment</li>
     *   <li>Triggers: Turbo/sniper speed modifiers</li>
     * </ul>
     *
     * @param swerve the swerve subsystem to control
     * @return the orbit command
     */
    public Command orbitCommand(SwerveSubsystem swerve) {
        return swerve.orbitCommand(
                xSupplier(),     // radial (forward/back = toward/away from target)
                ySupplier(),     // tangent (left/right = orbit around target)
                rotSupplier(),   // rotation trim
                sniperTrigger(),
                turboTrigger());
    }

    /**
     * Creates a face-target drive command using controller inputs.
     * <p>
     * Normal field-relative driving but the robot automatically rotates to face the target (Tower).
     * <ul>
     *   <li>Left stick: Normal translation (field-relative)</li>
     *   <li>Right stick X: Fine heading adjustment</li>
     *   <li>Triggers: Turbo/sniper speed modifiers</li>
     * </ul>
     *
     * @param swerve the swerve subsystem to control
     * @return the face-target drive command
     */
    public Command faceTargetDriveCommand(SwerveSubsystem swerve) {
        return swerve.faceTargetDriveCommand(
                xSupplier(),
                ySupplier(),
                rotSupplier(),
                sniperTrigger(),
                turboTrigger());
    }

    //region Button triggers ----------------------------------------------------

    /** @return trigger for A button (mapped for controller type) */
    public Trigger a() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_A_BUTTON : BITDO_A_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for B button (mapped for controller type) */
    public Trigger b() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_B_BUTTON : BITDO_B_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for X button (mapped for controller type) */
    public Trigger x() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_X_BUTTON : BITDO_X_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for Y button (mapped for controller type) */
    public Trigger y() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_Y_BUTTON : BITDO_Y_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for left bumper (mapped for controller type) */
    public Trigger leftBumper() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_LEFT_BUMPER : BITDO_LEFT_BUMPER;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for right bumper (mapped for controller type) */
    public Trigger rightBumper() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_RIGHT_BUMPER : BITDO_RIGHT_BUMPER;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for back/select button (mapped for controller type) */
    public Trigger back() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_BACK_BUTTON : BITDO_BACK_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for start button (mapped for controller type) */
    public Trigger start() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_START_BUTTON : BITDO_START_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for left stick click (mapped for controller type) */
    public Trigger leftStick() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_LEFT_STICK_BUTTON : BITDO_LEFT_STICK_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    /** @return trigger for right stick click (mapped for controller type) */
    public Trigger rightStick() {
        int button = useXboxMapping.getAsBoolean() ? XBOX_RIGHT_STICK_BUTTON : BITDO_RIGHT_STICK_BUTTON;
        return new Trigger(() -> controller.getHID().getRawButton(button));
    }

    //endregion
}

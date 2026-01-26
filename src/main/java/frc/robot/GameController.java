package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Config.Swerve.useXboxMapping;

/**
 * A controller abstraction that handles mapping between 8BitDo and Xbox controllers.
 * <p>
 * Supports two controller types via {@code Config.Swerve.useXboxMapping}:
 * <ul>
 *   <li>8BitDo (default): DirectInput mode mappings</li>
 *   <li>Xbox (for simulation): Standard WPILib XInput mappings</li>
 * </ul>
 * <p>
 * Use this instead of {@link CommandXboxController} to ensure consistent behavior
 * across controller types.
 */
public class GameController {

//region 8BitDo mapping --------------------------------------------------------

    private static final int BITDO_LEFT_X_AXIS = 0;
    private static final int BITDO_LEFT_Y_AXIS = 1;
    private static final int BITDO_RIGHT_X_AXIS = 2;
    private static final int BITDO_RIGHT_Y_AXIS = 3;
    private static final int BITDO_LEFT_TRIGGER_AXIS = 5;   // swapped in DirectInput mode
    private static final int BITDO_RIGHT_TRIGGER_AXIS = 4;  // swapped in DirectInput mode

    private static final int BITDO_A_BUTTON = 1;
    private static final int BITDO_B_BUTTON = 2;
    private static final int BITDO_X_BUTTON = 4;
    private static final int BITDO_Y_BUTTON = 5;
    private static final int BITDO_LEFT_BUMPER = 7;
    private static final int BITDO_RIGHT_BUMPER = 8;
    private static final int BITDO_BACK_BUTTON = 11;
    private static final int BITDO_START_BUTTON = 12;
    private static final int BITDO_LEFT_STICK_BUTTON = 14;
    private static final int BITDO_RIGHT_STICK_BUTTON = 15;

//endregion

//region Xbox mapping ----------------------------------------------------------

    private static final int XBOX_LEFT_X_AXIS = 0;
    private static final int XBOX_LEFT_Y_AXIS = 1;
    private static final int XBOX_RIGHT_X_AXIS = 4;
    private static final int XBOX_RIGHT_Y_AXIS = 5;
    private static final int XBOX_LEFT_TRIGGER_AXIS = 2;
    private static final int XBOX_RIGHT_TRIGGER_AXIS = 3;

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

//endregion

//region Implementation --------------------------------------------------------

    private final CommandXboxController controller;

    /**
     * Creates a {@link GameController}.
     *
     * @param port the HID port number (0-5)
     */
    public GameController(int port) {
        this.controller = new CommandXboxController(port);
    }

    /**
     * @return the underlying HID for direct access when needed
     */
    public GenericHID getHID() {
        return controller.getHID();
    }

    /**
     * @return the port number this controller is connected to
     */
    public int getPort() {
        return controller.getHID().getPort();
    }

    //endregion

    //region Axis suppliers ----------------------------------------------------

    /** @return the left stick X axis value (-1 to 1) */
    public double getLeftX() {
        int axis = useXboxMapping.getAsBoolean() ? XBOX_LEFT_X_AXIS : BITDO_LEFT_X_AXIS;
        return controller.getHID().getRawAxis(axis);
    }

    /** @return the left stick Y axis value (-1 to 1) */
    public double getLeftY() {
        int axis = useXboxMapping.getAsBoolean() ? XBOX_LEFT_Y_AXIS : BITDO_LEFT_Y_AXIS;
        return controller.getHID().getRawAxis(axis);
    }

    /** @return the right stick X axis value (-1 to 1) */
    public double getRightX() {
        int axis = useXboxMapping.getAsBoolean() ? XBOX_RIGHT_X_AXIS : BITDO_RIGHT_X_AXIS;
        return controller.getHID().getRawAxis(axis);
    }

    /** @return the right stick Y axis value (-1 to 1) */
    public double getRightY() {
        int axis = useXboxMapping.getAsBoolean() ? XBOX_RIGHT_Y_AXIS : BITDO_RIGHT_Y_AXIS;
        return controller.getHID().getRawAxis(axis);
    }

    /**
     * @return the left trigger axis value (0 to 1, normalized from controller-specific range)
     */
    public double getLeftTriggerAxis() {
        if (useXboxMapping.getAsBoolean()) {
            // Xbox triggers: 0 (not pressed) to 1 (fully pressed)
            return controller.getHID().getRawAxis(XBOX_LEFT_TRIGGER_AXIS);
        } else {
            // 8BitDo triggers: -1 (not pressed) to 1 (fully pressed), normalize to 0-1
            return (controller.getHID().getRawAxis(BITDO_LEFT_TRIGGER_AXIS) + 1) / 2;
        }
    }

    /**
     * @return the right trigger axis value (0 to 1, normalized from controller-specific range)
     */
    public double getRightTriggerAxis() {
        if (useXboxMapping.getAsBoolean()) {
            // Xbox triggers: 0 (not pressed) to 1 (fully pressed)
            return controller.getHID().getRawAxis(XBOX_RIGHT_TRIGGER_AXIS);
        } else {
            // 8BitDo triggers: -1 (not pressed) to 1 (fully pressed), normalize to 0-1
            return (controller.getHID().getRawAxis(BITDO_RIGHT_TRIGGER_AXIS) + 1) / 2;
        }
    }

    /** @return supplier for left stick X axis */
    public DoubleSupplier leftXSupplier() {
        return this::getLeftX;
    }

    /** @return supplier for left stick Y axis */
    public DoubleSupplier leftYSupplier() {
        return this::getLeftY;
    }

    /** @return supplier for right stick X axis */
    public DoubleSupplier rightXSupplier() {
        return this::getRightX;
    }

    /** @return supplier for right stick Y axis */
    public DoubleSupplier rightYSupplier() {
        return this::getRightY;
    }

    /** @return supplier for left trigger pulls */
    public BooleanSupplier leftTriggerSupplier() {
        return () -> getLeftTriggerAxis() > 0.5;
    }

    /** @return supplier for right trigger pulls */
    public BooleanSupplier rightTriggerSupplier() {
        return () -> getRightTriggerAxis() > 0.5;
    }

//endregion

//region Triggers --------------------------------------------------------------

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

    /** @return trigger for left trigger as button (threshold 0.5) */
    public Trigger leftTrigger() {
        return new Trigger(() -> getLeftTriggerAxis() > 0.5);
    }

    /** @return trigger for right trigger as button (threshold 0.5) */
    public Trigger rightTrigger() {
        return new Trigger(() -> getRightTriggerAxis() > 0.5);
    }

    /** @return trigger for D-pad up */
    public Trigger povUp() {
        return pov(0);
    }

    /** @return trigger for D-pad right */
    public Trigger povRight() {
        return pov(90);
    }

    /** @return trigger for D-pad down */
    public Trigger povDown() {
        return pov(180);
    }

    /** @return trigger for D-pad left */
    public Trigger povLeft() {
        return pov(270);
    }

    /**
     * @param angle the POV angle (0, 45, 90, 135, 180, 225, 270, 315)
     * @return trigger for specific D-pad angle
     */
    public Trigger pov(int angle) {
        return new Trigger(() -> controller.getHID().getPOV() == angle);
    }

//endregion

}

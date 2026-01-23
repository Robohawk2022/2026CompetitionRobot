package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Utility class that hooks into WPILib's {@link CommandScheduler} to log
 * command lifecycle events (initialize, finish, interrupt).
 * <p>
 * Call {@link #enable()} once in RobotContainer to start logging.
 * Logging respects the {@link Config.Logging} preferences.
 */
public class CommandLogger {

    /** Prefix for log messages */
    private static final String LOG_PREFIX = "[CMD] ";

    /** Singleton - only enable once */
    private static boolean enabled = false;

    /** Total commands run this session */
    private static int totalCommandsRun = 0;

    /** Currently active command names (ordered by start time) */
    private static final Set<String> activeCommands = new LinkedHashSet<>();

    /** Track command start times for duration logging */
    private static final Map<Command, Double> commandStartTimes = new HashMap<>();

    /** Counter for rate-limiting execute logging */
    private static int executeLogCounter = 0;

    /** How often to log execute calls (50 cycles = 1 second at 50Hz) */
    private static final int EXECUTE_LOG_INTERVAL = 50;

    /** Prefix for button log messages */
    private static final String BTN_PREFIX = "[BTN] ";

    /** Tracked controllers with their names and previous button states */
    private static final List<TrackedController> trackedControllers = new ArrayList<>();

    /** Trigger threshold for considering it "pressed" */
    private static final double TRIGGER_THRESHOLD = 0.5;

    /** Button names for Xbox controllers (by button index) */
    private static final String[] XBOX_BUTTON_NAMES = {
            "A",           // 0 - kA
            "B",           // 1 - kB
            "X",           // 2 - kX
            "Y",           // 3 - kY
            "LBumper",     // 4 - kLeftBumper
            "RBumper",     // 5 - kRightBumper
            "Back",        // 6 - kBack
            "Start",       // 7 - kStart
            "LStick",      // 8 - kLeftStick (joystick button)
            "RStick"       // 9 - kRightStick (joystick button)
    };

    /** POV/D-pad direction names */
    private static final String[] POV_NAMES = {
            "Up", "UpRight", "Right", "DownRight",
            "Down", "DownLeft", "Left", "UpLeft"
    };

    private CommandLogger() {
        // utility class - prevent instantiation
    }

    /**
     * Internal class to track a controller's button states.
     */
    private static class TrackedController {
        final String name;
        final GenericHID hid;
        final boolean[] prevButtons;
        int prevPOV = -1;
        boolean prevLeftTrigger = false;
        boolean prevRightTrigger = false;

        TrackedController(String name, GenericHID hid, int buttonCount) {
            this.name = name;
            this.hid = hid;
            this.prevButtons = new boolean[buttonCount];
        }
    }

    /**
     * Enable command logging. Call once in RobotContainer constructor.
     * <p>
     * Registers callbacks with {@link CommandScheduler} to log:
     * <ul>
     *   <li>Command initialization</li>
     *   <li>Command execution (rate-limited, if enabled)</li>
     *   <li>Command finish</li>
     *   <li>Command interrupt</li>
     * </ul>
     */
    public static void enable() {
        if (enabled) {
            return;
        }
        enabled = true;

        CommandScheduler scheduler = CommandScheduler.getInstance();

        scheduler.onCommandInitialize(CommandLogger::onInitialize);
        scheduler.onCommandExecute(CommandLogger::onExecute);
        scheduler.onCommandFinish(CommandLogger::onFinish);
        scheduler.onCommandInterrupt(CommandLogger::onInterrupt);

        setupTelemetry();

        Util.log("%sCommand logging enabled", LOG_PREFIX);
    }

    /**
     * Register an Xbox controller for button logging.
     * Call after enable() in RobotContainer.
     *
     * @param name display name for this controller (e.g., "Driver", "Operator")
     * @param controller the CommandXboxController to track
     */
    public static void addController(String name, CommandXboxController controller) {
        addController(name, controller.getHID());
    }

    /**
     * Register an Xbox controller for button logging.
     * Call after enable() in RobotContainer.
     *
     * @param name display name for this controller (e.g., "Driver", "Operator")
     * @param controller the XboxController to track
     */
    public static void addController(String name, XboxController controller) {
        addController(name, (GenericHID) controller);
    }

    /**
     * Register any GenericHID for button logging.
     *
     * @param name display name for this controller
     * @param hid the GenericHID to track
     */
    public static void addController(String name, GenericHID hid) {
        // Xbox controllers have 10 buttons
        trackedControllers.add(new TrackedController(name, hid, 10));
        Util.log("%sTracking controller: %s (port %d)", BTN_PREFIX, name, hid.getPort());
    }

    /**
     * Poll all registered controllers for button changes.
     * Call this method from robotPeriodic() in Robot.java.
     */
    public static void pollButtons() {
        if (!isButtonLoggingEnabled()) {
            return;
        }

        for (TrackedController tc : trackedControllers) {
            pollControllerButtons(tc);
        }
    }

    /**
     * Poll a single controller for button changes.
     */
    private static void pollControllerButtons(TrackedController tc) {
        GenericHID hid = tc.hid;

        // Check all regular buttons (including joystick buttons at indices 8 and 9)
        for (int i = 0; i < tc.prevButtons.length; i++) {
            boolean current = hid.getRawButton(i + 1); // buttons are 1-indexed in WPILib
            if (current != tc.prevButtons[i]) {
                String buttonName = (i < XBOX_BUTTON_NAMES.length) ? XBOX_BUTTON_NAMES[i] : "Button" + (i + 1);
                logButtonChange(tc.name, buttonName, current);
                tc.prevButtons[i] = current;
            }
        }

        // Check POV (D-pad)
        int currentPOV = hid.getPOV();
        if (currentPOV != tc.prevPOV) {
            if (tc.prevPOV != -1) {
                String prevDir = getPOVName(tc.prevPOV);
                logButtonChange(tc.name, "POV-" + prevDir, false);
            }
            if (currentPOV != -1) {
                String currDir = getPOVName(currentPOV);
                logButtonChange(tc.name, "POV-" + currDir, true);
            }
            tc.prevPOV = currentPOV;
        }

        // Check triggers (treat as buttons with threshold)
        if (hid instanceof XboxController xbox) {
            boolean leftTrigger = xbox.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
            boolean rightTrigger = xbox.getRightTriggerAxis() > TRIGGER_THRESHOLD;

            if (leftTrigger != tc.prevLeftTrigger) {
                logButtonChange(tc.name, "LTrigger", leftTrigger);
                tc.prevLeftTrigger = leftTrigger;
            }
            if (rightTrigger != tc.prevRightTrigger) {
                logButtonChange(tc.name, "RTrigger", rightTrigger);
                tc.prevRightTrigger = rightTrigger;
            }
        }
    }

    /**
     * Get POV direction name from angle.
     */
    private static String getPOVName(int angle) {
        int index = angle / 45;
        if (index >= 0 && index < POV_NAMES.length) {
            return POV_NAMES[index];
        }
        return String.valueOf(angle);
    }

    /**
     * Log a button state change.
     */
    private static void logButtonChange(String controller, String button, boolean pressed) {
        String action = pressed ? "PRESSED" : "RELEASED";
        Util.log("%s%s %s: %s", BTN_PREFIX, controller, button, action);
    }

    /**
     * @return true if button logging is enabled via preferences
     */
    private static boolean isButtonLoggingEnabled() {
        return Config.Logging.isEnabled(Config.Logging.controllerLogging);
    }

    /**
     * @return true if command logging is enabled via preferences
     */
    private static boolean isLoggingEnabled() {
        return Config.Logging.isEnabled(Config.Logging.commandLogging);
    }

    /**
     * @return true if execute logging is enabled via preferences
     */
    private static boolean isExecuteLoggingEnabled() {
        return Config.Logging.isEnabled(Config.Logging.commandExecuteLogging);
    }

    /**
     * Called when a command is initialized (starts running).
     */
    private static void onInitialize(Command cmd) {
        String name = cmd.getName();
        activeCommands.add(name);
        commandStartTimes.put(cmd, Timer.getFPGATimestamp());
        totalCommandsRun++;

        if (isLoggingEnabled()) {
            String requirements = formatRequirements(cmd);
            Util.log("%sINIT: %s%s", LOG_PREFIX, name, requirements);
        }
    }

    /**
     * Called every cycle while a command is running.
     * Rate-limited to avoid log spam.
     */
    private static void onExecute(Command cmd) {
        if (!isExecuteLoggingEnabled()) {
            return;
        }

        executeLogCounter++;
        if (executeLogCounter >= EXECUTE_LOG_INTERVAL) {
            executeLogCounter = 0;
            Util.log("%sEXECUTE: (%d active) %s",
                    LOG_PREFIX,
                    activeCommands.size(),
                    String.join(", ", activeCommands));
        }
    }

    /**
     * Called when a command finishes normally.
     */
    private static void onFinish(Command cmd) {
        String name = cmd.getName();
        activeCommands.remove(name);
        String duration = formatDuration(cmd);
        commandStartTimes.remove(cmd);

        if (isLoggingEnabled()) {
            Util.log("%sFINISH: %s%s", LOG_PREFIX, name, duration);
        }
    }

    /**
     * Called when a command is interrupted.
     */
    private static void onInterrupt(Command cmd) {
        String name = cmd.getName();
        activeCommands.remove(name);
        String duration = formatDuration(cmd);
        commandStartTimes.remove(cmd);

        if (isLoggingEnabled()) {
            Util.log("%sINTERRUPT: %s%s", LOG_PREFIX, name, duration);
        }
    }

    /**
     * Format the requirements (subsystems) for a command.
     */
    private static String formatRequirements(Command cmd) {
        var requirements = cmd.getRequirements();
        if (requirements.isEmpty()) {
            return "";
        }

        String names = requirements.stream()
                .map(Subsystem::getName)
                .collect(Collectors.joining(", "));

        return " [" + names + "]";
    }

    /**
     * Format the duration a command ran for.
     */
    private static String formatDuration(Command cmd) {
        Double startTime = commandStartTimes.get(cmd);
        if (startTime == null) {
            return "";
        }

        double duration = Timer.getFPGATimestamp() - startTime;
        return String.format(" (ran for %.2fs)", duration);
    }

    /**
     * Setup SmartDashboard telemetry.
     */
    private static void setupTelemetry() {
        SmartDashboard.putData("CommandLogger", builder -> {
            builder.addIntegerProperty("ActiveCount",
                    activeCommands::size, null);
            builder.addStringProperty("ActiveCommands",
                    () -> String.join(", ", activeCommands), null);
            builder.addIntegerProperty("TotalRun",
                    () -> totalCommandsRun, null);
            builder.addBooleanProperty("Enabled",
                    CommandLogger::isLoggingEnabled, null);
        });
    }
}

package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDHardwareWPILib;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LEDColor;

/**
 * Standalone test program for the LED subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LEDTestbot}
 * <p>
 * <b>Testing with Elastic:</b>
 * <ol>
 *   <li>Start Elastic and connect to the simulation</li>
 *   <li>Add the "LEDSubsystem" widget to see current mode, pattern, and color</li>
 *   <li>Use the "LED/ColorChooser" and "LED/PatternChooser" to change settings</li>
 *   <li>Or use controller buttons to cycle through colors and patterns</li>
 * </ol>
 */
public class LEDTestbot extends TimedRobot {

    private LEDSubsystem led;
    private CommandXboxController controller;

    private SendableChooser<LEDColor> colorChooser;
    private SendableChooser<String> patternChooser;

    private LEDColor currentColor = LEDColor.BLUE;
    private String currentPattern = "Solid";

    @Override
    public void robotInit() {
        System.out.println(">>> LEDTestbot starting...");

        // Use appropriate hardware based on environment
        led = new LEDSubsystem(
                isSimulation() ? new LEDHardwareSim() : new LEDHardwareWPILib());
        controller = new CommandXboxController(0);

        // Create color chooser for Elastic
        colorChooser = new SendableChooser<>();
        for (LEDColor color : LEDColor.values()) {
            if (color == LEDColor.BLUE) {
                colorChooser.setDefaultOption(color.name(), color);
            } else {
                colorChooser.addOption(color.name(), color);
            }
        }
        SmartDashboard.putData("LED/ColorChooser", colorChooser);

        // Create pattern chooser for Elastic
        patternChooser = new SendableChooser<>();
        patternChooser.setDefaultOption("Solid", "Solid");
        patternChooser.addOption("Blink", "Blink");
        patternChooser.addOption("Rainbow", "Rainbow");
        patternChooser.addOption("Breathing", "Breathing");
        patternChooser.addOption("Chase", "Chase");
        patternChooser.addOption("Off", "Off");
        SmartDashboard.putData("LED/PatternChooser", patternChooser);

        // Button bindings - use Commands.runOnce to avoid subsystem conflicts
        // A = cycle colors forward
        controller.a().onTrue(Commands.runOnce(() -> {
            LEDColor[] colors = LEDColor.values();
            int nextIndex = (currentColor.ordinal() + 1) % colors.length;
            currentColor = colors[nextIndex];
            System.out.println(">>> Color: " + currentColor.name());
        }));

        // B = cycle patterns
        controller.b().onTrue(Commands.runOnce(() -> {
            String[] patterns = {"Solid", "Blink", "Rainbow", "Breathing", "Chase", "Off"};
            int currentIndex = 0;
            for (int i = 0; i < patterns.length; i++) {
                if (patterns[i].equals(currentPattern)) {
                    currentIndex = i;
                    break;
                }
            }
            currentPattern = patterns[(currentIndex + 1) % patterns.length];
            System.out.println(">>> Pattern: " + currentPattern);
        }));

        // Bumpers for alliance colors (override current settings while held)
        controller.leftBumper()
            .onTrue(Commands.runOnce(() -> currentColor = LEDColor.ALLIANCE_BLUE))
            .onFalse(Commands.runOnce(() -> currentColor = colorChooser.getSelected()));
        controller.rightBumper()
            .onTrue(Commands.runOnce(() -> currentColor = LEDColor.ALLIANCE_RED))
            .onFalse(Commands.runOnce(() -> currentColor = colorChooser.getSelected()));

        // Triggers for special patterns (override while held)
        controller.leftTrigger()
            .onTrue(Commands.runOnce(() -> currentPattern = "Rainbow"))
            .onFalse(Commands.runOnce(() -> currentPattern = patternChooser.getSelected()));
        controller.rightTrigger()
            .onTrue(Commands.runOnce(() -> { currentPattern = "Chase"; currentColor = LEDColor.GREEN; }))
            .onFalse(Commands.runOnce(() -> {
                currentPattern = patternChooser.getSelected();
                currentColor = colorChooser.getSelected();
            }));

        // Y = off
        controller.y().onTrue(Commands.runOnce(() -> currentPattern = "Off"));

        // X = breathing with current color (while held)
        controller.x()
            .onTrue(Commands.runOnce(() -> currentPattern = "Breathing"))
            .onFalse(Commands.runOnce(() -> currentPattern = patternChooser.getSelected()));

        System.out.println(">>> Button mappings:");
        System.out.println("    A = Cycle colors");
        System.out.println("    B = Cycle patterns");
        System.out.println("    X (hold) = Breathing effect");
        System.out.println("    Y = Off");
        System.out.println("    LB (hold) = Blue alliance");
        System.out.println("    RB (hold) = Red alliance");
        System.out.println("    LT (hold) = Rainbow");
        System.out.println("    RT (hold) = Green chase");
        System.out.println("");
        System.out.println(">>> Use Elastic choosers to select color/pattern!");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Sync chooser selections to current settings (when not overridden by buttons)
        LEDColor selectedColor = colorChooser.getSelected();
        String selectedPattern = patternChooser.getSelected();

        if (selectedColor != null && selectedColor != currentColor) {
            if (!controller.getHID().getLeftBumper() &&
                !controller.getHID().getRightBumper() &&
                controller.getHID().getRightTriggerAxis() < 0.5) {
                currentColor = selectedColor;
            }
        }
        if (selectedPattern != null && !selectedPattern.equals(currentPattern)) {
            if (controller.getHID().getLeftTriggerAxis() < 0.5 &&
                controller.getHID().getRightTriggerAxis() < 0.5 &&
                !controller.getHID().getXButton()) {
                currentPattern = selectedPattern;
            }
        }

        // Apply current pattern using direct control methods
        applyCurrentSettings();
    }

    /**
     * Applies the current color and pattern settings to the LEDs.
     * Uses direct control methods (not commands) for immediate updates.
     */
    private void applyCurrentSettings() {
        switch (currentPattern) {
            case "Solid":
                led.setSolid(currentColor);
                break;
            case "Blink":
                led.setBlink(currentColor);
                break;
            case "Rainbow":
                led.setRainbow();
                break;
            case "Breathing":
                led.setBreathing(currentColor);
                break;
            case "Chase":
                led.setChase(currentColor);
                break;
            case "Off":
                led.setOff();
                break;
        }
    }

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}
}

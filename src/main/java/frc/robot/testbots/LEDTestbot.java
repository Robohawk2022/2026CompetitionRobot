package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.led.LEDHardwareBlinkin;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;

/**
 * Standalone test program for the LED subsystem using REV Blinkin.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LEDTestbot}
 * <p>
 * <b>Testing with Elastic:</b>
 * <ol>
 *   <li>Start Elastic and connect to the simulation</li>
 *   <li>Add the "LEDSubsystem" widget to see current signal and Blinkin code</li>
 *   <li>Add the "LED/SimSignal" string to see the signal name in simulation</li>
 *   <li>Use the "LED/SignalChooser" dropdown to select signals</li>
 *   <li>Or use controller buttons to test common signals</li>
 * </ol>
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A - Intaking (green)</li>
 *   <li>B - Intake Full (red strobe)</li>
 *   <li>X - Ready to Shoot (gold strobe)</li>
 *   <li>Y - Off</li>
 *   <li>LB (hold) - Alliance Blue</li>
 *   <li>RB (hold) - Alliance Red</li>
 *   <li>LT (hold) - Rainbow/Celebration</li>
 *   <li>RT (hold) - Error signal</li>
 *   <li>D-pad Up - Target Locked</li>
 *   <li>D-pad Down - Target Searching</li>
 * </ul>
 */
public class LEDTestbot extends TimedRobot {

    private LEDSubsystem led;
    private CommandXboxController controller;
    private SendableChooser<LEDSignal> signalChooser;
    private LEDSignal currentSignal = LEDSignal.IDLE;

    @Override
    public void robotInit() {
        System.out.println(">>> LEDTestbot starting...");

        // Use appropriate hardware based on environment
        led = new LEDSubsystem(
                isSimulation() ? new LEDHardwareSim() : new LEDHardwareBlinkin());
        controller = new CommandXboxController(0);

        // Create signal chooser for Elastic
        signalChooser = new SendableChooser<>();
        signalChooser.setDefaultOption("IDLE", LEDSignal.IDLE);
        for (LEDSignal signal : LEDSignal.values()) {
            if (signal != LEDSignal.IDLE) {
                signalChooser.addOption(signal.name(), signal);
            }
        }
        SmartDashboard.putData("LED/SignalChooser", signalChooser);

        // Button bindings for common signals
        controller.a().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.INTAKING)));
        controller.b().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.INTAKE_FULL)));
        controller.x().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.READY_TO_SHOOT)));
        controller.y().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.OFF)));

        // Bumpers for alliance colors (while held)
        controller.leftBumper()
            .onTrue(Commands.runOnce(() -> setSignal(LEDSignal.ALLIANCE_BLUE)))
            .onFalse(Commands.runOnce(() -> setSignal(signalChooser.getSelected())));
        controller.rightBumper()
            .onTrue(Commands.runOnce(() -> setSignal(LEDSignal.ALLIANCE_RED)))
            .onFalse(Commands.runOnce(() -> setSignal(signalChooser.getSelected())));

        // Triggers for special effects (while held)
        controller.leftTrigger()
            .onTrue(Commands.runOnce(() -> setSignal(LEDSignal.CELEBRATION)))
            .onFalse(Commands.runOnce(() -> setSignal(signalChooser.getSelected())));
        controller.rightTrigger()
            .onTrue(Commands.runOnce(() -> setSignal(LEDSignal.ERROR)))
            .onFalse(Commands.runOnce(() -> setSignal(signalChooser.getSelected())));

        // D-pad for targeting signals
        controller.povUp().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.TARGET_LOCKED)));
        controller.povDown().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.TARGET_SEARCHING)));
        controller.povLeft().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.SHOOT_RANGE_FAR)));
        controller.povRight().onTrue(Commands.runOnce(() -> setSignal(LEDSignal.SHOOT_RANGE_CLOSE)));

        System.out.println(">>> Button mappings:");
        System.out.println("    A = Intaking (green)");
        System.out.println("    B = Intake Full (red strobe)");
        System.out.println("    X = Ready to Shoot (gold strobe)");
        System.out.println("    Y = Off");
        System.out.println("    LB (hold) = Alliance Blue");
        System.out.println("    RB (hold) = Alliance Red");
        System.out.println("    LT (hold) = Celebration (rainbow)");
        System.out.println("    RT (hold) = Error (red strobe)");
        System.out.println("    D-pad Up = Target Locked");
        System.out.println("    D-pad Down = Target Searching");
        System.out.println("    D-pad Left = Shoot Range Far");
        System.out.println("    D-pad Right = Shoot Range Close");
        System.out.println("");
        System.out.println(">>> Use Elastic 'LED/SignalChooser' to test any signal!");
    }

    private void setSignal(LEDSignal signal) {
        currentSignal = signal;
        led.setSignal(signal);
        System.out.println(">>> Signal: " + signal.name() + " (code: " + signal.getBlinkinCode() + ")");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Sync chooser selection (when not overridden by buttons)
        LEDSignal selected = signalChooser.getSelected();
        if (selected != null && selected != currentSignal) {
            // Only update if no buttons are being held
            if (!controller.getHID().getLeftBumper() &&
                !controller.getHID().getRightBumper() &&
                controller.getHID().getLeftTriggerAxis() < 0.5 &&
                controller.getHID().getRightTriggerAxis() < 0.5) {
                setSignal(selected);
            }
        }
    }

    @Override
    public void disabledInit() {
        led.setSignal(LEDSignal.DISABLED);
    }

    @Override
    public void teleopInit() {
        led.setSignal(LEDSignal.IDLE);
    }

    @Override
    public void autonomousInit() {
        led.setSignal(LEDSignal.AUTO_MODE);
    }
}

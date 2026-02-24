package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.led.LEDHardwareBlinkin;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.util.Util;

/**
 * Standalone test program for the LED subsystem using REV Blinkin
 */
public class LEDTestbot extends TimedRobot {

    public static final int LED_PWM_PORT = 0;

    private LEDSubsystem led;
    private SendableChooser<LEDSignal> signalChooser;

    @Override
    public void robotInit() {

        // Use appropriate hardware based on environment
        led = new LEDSubsystem(isSimulation()
                ? new LEDHardwareSim()
                : new LEDHardwareBlinkin(LED_PWM_PORT));
        led.setDefaultCommand(led.show(LEDSignal.OFF));

        GameController controller = new GameController(0);

        // Create signal chooser for Elastic
        signalChooser = createSignalChooser();
        SmartDashboard.putData("LED/SignalChooser", signalChooser);

        // Button bindings for common signals
        controller.a().whileTrue(led.show(LEDSignal.INTAKING));
        controller.b().whileTrue(led.show(LEDSignal.INTAKE_FULL));
        controller.x().whileTrue(led.show(LEDSignal.READY_TO_SHOOT));
        controller.y().whileTrue(led.defer(() -> led.show(signalChooser.getSelected())));
    }

    private SendableChooser<LEDSignal> createSignalChooser() {
        SendableChooser<LEDSignal> chooser = new SendableChooser<>();
        chooser.setDefaultOption("IDLE", LEDSignal.IDLE);
        for (LEDSignal signal : LEDSignal.values()) {
            if (signal != LEDSignal.IDLE) {
                chooser.addOption(signal.name(), signal);
            }
        }
        return chooser;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        led.show(LEDSignal.DISABLED);
    }

    @Override
    public void teleopInit() {
        LEDSignal signal = Util.isRedAlliance()
                ? LEDSignal.ALLIANCE_RED
                : LEDSignal.ALLIANCE_BLUE;
        CommandScheduler.getInstance().schedule(led.flash(signal));
    }

    @Override
    public void autonomousInit() {
        led.show(LEDSignal.AUTO_MODE);
    }
}
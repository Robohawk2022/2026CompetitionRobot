package frc.robot.subsystems.led;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling LEDs via the REV Blinkin controller.
 * <p>
 * LED priority (highest to lowest):
 * <ol>
 *   <li>Timed flash (alliance change, intake full, etc.)</li>
 *   <li>Distance-to-hub indicator (green = in range, red = out of range)</li>
 * </ol>
 * <p>
 * Use {@link #flash(LEDSignal, double)} to temporarily override the distance
 * display with a higher-priority signal.
 */
public class LEDSubsystem extends SubsystemBase {

    public static final double FLASH_CYCLE_DURATION = 0.4;
    public static final int FLASH_CYCLE_COUNT = 4;

    private final LEDHardware hardware;
    private LEDSignal currentSignal;

    /**
     * Creates a {@link LEDSubsystem}.
     * @param hardware the LED hardware interface (required)
     */
    public LEDSubsystem(LEDHardware hardware) {

        this.hardware = Objects.requireNonNull(hardware);
        this.currentSignal = LEDSignal.OFF;
        applySignal(LEDSignal.OFF);

        // Dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Signal", () -> currentSignal.name(), null);
            builder.addDoubleProperty("Code", () -> currentSignal.getBlinkinCode(), null);
        });
    }

    private void applySignal(LEDSignal signal) {
        currentSignal = signal;
        hardware.applySignal(signal);
    }

    /**
     * @return a command that will show the supplied signal until interrupted
     */
    public Command show(LEDSignal signal) {
        return run(() -> applySignal(signal));
    }

    /**
     * @return a command that will show the supplied signal for the specified
     * number of seconds, or until interrupted
     */
    public Command showWithTimeout(LEDSignal signal, double timeout) {
        return show(signal).withTimeout(timeout);
    }

    /**
     * @return a command that will flash the supplied signal for a short
     * period, or until interrupted
     */
    public Command flash(LEDSignal signal) {
        Command cmd = showWithTimeout(signal, FLASH_CYCLE_DURATION);
        for (int i=1; i<FLASH_CYCLE_COUNT; i++) {
            cmd = cmd.andThen(showWithTimeout(LEDSignal.OFF, FLASH_CYCLE_DURATION));
            cmd = cmd.andThen(showWithTimeout(signal, FLASH_CYCLE_DURATION));
        }
        return cmd;
    }
}

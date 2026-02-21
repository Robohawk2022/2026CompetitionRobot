package frc.robot.subsystems.led;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.Shooting.*;

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

    //region Implementation --------------------------------------------------------

    /** How long to flash alliance color in seconds */
    static final double ALLIANCE_FLASH_DURATION = 3.0;

    private final LEDHardware hardware;
    private DoubleSupplier distanceSupplier;
    private LEDSignal currentSignal = LEDSignal.OFF;
    private String shotZone = "none";

    // flash tracking (alliance flash, intake full, etc.)
    private final Timer flashTimer = new Timer();
    private LEDSignal flashSignal = LEDSignal.OFF;
    private double flashDuration = 0;
    private boolean flashing = false;

    // alliance change detection
    private boolean lastAllianceWasRed;

    /**
     * Creates a {@link LEDSubsystem}.
     *
     * @param hardware the LED hardware interface (required)
     */
    public LEDSubsystem(LEDHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);

        // flash alliance color on startup
        lastAllianceWasRed = Util.isRedAlliance();
        flash(lastAllianceWasRed ? LEDSignal.ALLIANCE_RED : LEDSignal.ALLIANCE_BLUE,
                ALLIANCE_FLASH_DURATION);

        // Dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Signal", () -> currentSignal.name(), null);
            builder.addStringProperty("ShotZone", () -> shotZone, null);
            builder.addBooleanProperty("Flashing?", () -> flashing, null);
            builder.addDoubleProperty("BlinkinCode", () -> currentSignal.getBlinkinCode(), null);
        });
    }

    /**
     * Sets a supplier that provides the distance to the hub in feet.
     * When set, the LED will automatically show green when in a shooting
     * zone and red when out of range.
     *
     * @param distanceSupplier supplies distance to hub in feet
     */
    public void setDistanceSupplier(DoubleSupplier distanceSupplier) {
        this.distanceSupplier = distanceSupplier;
    }

    /**
     * Temporarily flashes a signal, overriding the distance display.
     * After the duration expires, the LED returns to showing distance.
     *
     * @param signal the signal to flash
     * @param durationSeconds how long to flash
     */
    public void flash(LEDSignal signal, double durationSeconds) {
        flashSignal = signal;
        flashDuration = durationSeconds;
        flashing = true;
        flashTimer.restart();
        Util.log("[led] flash: %s for %.1f sec", signal.name(), durationSeconds);
    }

    @Override
    public void periodic() {

        // check if alliance changed
        boolean currentlyRed = Util.isRedAlliance();
        if (currentlyRed != lastAllianceWasRed) {
            lastAllianceWasRed = currentlyRed;
            flash(currentlyRed ? LEDSignal.ALLIANCE_RED : LEDSignal.ALLIANCE_BLUE,
                    ALLIANCE_FLASH_DURATION);
        }

        // priority 1: timed flash
        if (flashing) {
            if (flashTimer.get() < flashDuration) {
                setSignal(flashSignal);
                return;
            }
            flashing = false;
            flashTimer.stop();
        }

        // priority 2: distance to hub
        if (distanceSupplier != null) {
            updateDistanceSignal();
        }
    }

    private void updateDistanceSignal() {
        double distance = distanceSupplier.getAsDouble();
        double tol = distanceTolerance.getAsDouble();

        if (Math.abs(distance - closeDistance.getAsDouble()) <= tol) {
            shotZone = "close";
            setSignal(LEDSignal.SHOOT_RANGE_CLOSE);
        } else if (Math.abs(distance - farDistance.getAsDouble()) <= tol) {
            shotZone = "far";
            setSignal(LEDSignal.SHOOT_RANGE_CLOSE);
        } else {
            shotZone = "none";
            setSignal(LEDSignal.SHOOT_RANGE_FAR);
        }
    }

    /**
     * @return the current shot zone ("close", "far", or "none")
     */
    public String getShotZone() {
        return shotZone;
    }

    /**
     * @return the currently displayed signal
     */
    public LEDSignal getCurrentSignal() {
        return currentSignal;
    }

    //endregion

    //region Command factories -----------------------------------------------------

    /**
     * @param signal the signal to display
     * @return a command that sets the LED signal (runs once)
     */
    public Command signalCommand(LEDSignal signal) {
        return runOnce(() -> setSignal(signal))
            .withName("LED:" + signal.name());
    }

    /**
     * @param signal the signal to display while held
     * @return a command that displays a signal while running, then turns off
     */
    public Command signalWhileCommand(LEDSignal signal) {
        return startEnd(
            () -> setSignal(signal),
            () -> setSignal(LEDSignal.OFF)
        ).withName("LEDWhile:" + signal.name());
    }

    /**
     * @return a command that turns off the LEDs
     */
    public Command offCommand() {
        return runOnce(() -> setSignal(LEDSignal.OFF))
            .withName("LED:OFF");
    }

    /**
     * @return a command that does nothing (maintains current state)
     */
    public Command idleCommand() {
        return run(() -> {
            // Keep current state, do nothing
        }).withName("LED:Idle");
    }

    //endregion

    //region Direct control methods ------------------------------------------------

    /**
     * Directly sets the LED signal (for use outside command system).
     * <p>
     * Prefer using {@link #signalCommand(LEDSignal)} when possible.
     *
     * @param signal the signal to display
     */
    public void setSignal(LEDSignal signal) {
        if (signal == null) {
            signal = LEDSignal.OFF;
        }
        currentSignal = signal;
        hardware.applySignal(signal);
    }

    /**
     * Directly turns off the LEDs.
     */
    public void off() {
        setSignal(LEDSignal.OFF);
    }

    //endregion
}

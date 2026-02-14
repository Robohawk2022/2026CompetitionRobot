package frc.robot.subsystems.agitator;

/**
 * Interface for the agitator motor hardware.
 * <p>
 * Single NEO motor that feeds balls toward or away from the launcher wheels.
 * Open-loop voltage control.
 */
public interface AgitatorHardware {

    /**
     * Applies voltage to the agitator motor.
     *
     * @param volts voltage to apply (positive = toward wheels)
     */
    void applyVolts(double volts);

    /** @return motor velocity in RPM */
    double getRPM();

    /** @return motor output current in amps */
    double getAmps();

    /** Stops the motor */
    default void stop() {
        applyVolts(0);
    }
}

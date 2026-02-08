package frc.robot.subsystems.launcher;

/**
 * Interface for the hardware of a launcher subsystem with 3 NEO motors:
 * agitator, lower wheel, and upper wheel.
 * <p>
 * The lower wheel doubles as intake by spinning backward.
 * Wheels use onboard closed-loop velocity control (SparkMax PID).
 * Agitator uses open-loop voltage control.
 */
public interface LauncherHardware {

    /**
     * Sets the target RPM for the lower wheel using onboard velocity PID.
     *
     * @param rpm target RPM (negative = reverse for intake)
     */
    void setLowerWheelRPM(double rpm);

    /**
     * Sets the target RPM for the upper wheel using onboard velocity PID.
     *
     * @param rpm target RPM (negative = reverse)
     */
    void setUpperWheelRPM(double rpm);

    /** Applies voltage to the agitator motor (open-loop) */
    void applyAgitatorVolts(double volts);

    /**
     * Resets/reconfigures the onboard velocity PID with current gains.
     * Call this when starting a command to pick up live-tuned values.
     *
     * @param kV feedforward gain (volts per RPM)
     * @param kP proportional gain
     * @param kD derivative gain
     */
    void resetPID(double kV, double kP, double kD);

    /** @return agitator motor velocity in RPM */
    double getAgitatorRPM();

    /** @return lower wheel motor velocity in RPM */
    double getLowerWheelRPM();

    /** @return upper wheel motor velocity in RPM */
    double getUpperWheelRPM();

    /** @return agitator motor current in amps */
    double getAgitatorAmps();

    /** @return lower wheel motor current in amps */
    double getLowerWheelAmps();

    /** @return upper wheel motor current in amps */
    double getUpperWheelAmps();

    /** Stops all motors */
    default void stopAll() {
        setLowerWheelRPM(0);
        setUpperWheelRPM(0);
        applyAgitatorVolts(0);
    }
}

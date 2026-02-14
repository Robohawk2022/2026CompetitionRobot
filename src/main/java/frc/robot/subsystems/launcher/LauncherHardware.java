package frc.robot.subsystems.launcher;

/**
 * Interface for the launcher wheel hardware: lower wheel and upper wheel.
 * <p>
 * Both wheels use onboard closed-loop velocity control (SparkMax PID).
 * The lower wheel doubles as intake by spinning backward.
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

    /**
     * Resets/reconfigures the lower wheel onboard velocity PID.
     * Call this when starting a command to pick up live-tuned values.
     */
    void resetLowerPID(double kV, double kP);

    /**
     * Resets/reconfigures the upper wheel onboard velocity PID.
     * Call this when starting a command to pick up live-tuned values.
     */
    void resetUpperPID(double kV, double kP);

    /** @return lower wheel motor velocity in RPM */
    double getLowerWheelRPM();

    /** @return upper wheel motor velocity in RPM */
    double getUpperWheelRPM();

    /** @return lower wheel motor current in amps */
    double getLowerWheelAmps();

    /** @return upper wheel motor current in amps */
    double getUpperWheelAmps();

    /** Stops both wheels */
    default void stopAll() {
        setLowerWheelRPM(0);
        setUpperWheelRPM(0);
    }
}

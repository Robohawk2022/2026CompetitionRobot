package frc.robot.subsystems.launcher;

/**
 * Interface for the launcher hardware: 2 feeder motors + 1 shooter motor.
 * <p>
 * All motors use onboard closed-loop velocity control (SparkMax PID).
 * Feeders spin inward to pull balls in; shooter flings them out.
 */
public interface LauncherHardware {

    /**
     * Sets the target RPM for the left feeder using onboard velocity PID.
     *
     * @param rpm target RPM (positive = inward, negative = outward)
     */
    void setFeederLeftRPM(double rpm);

    /**
     * Sets the target RPM for the right feeder using onboard velocity PID.
     *
     * @param rpm target RPM (positive = inward, negative = outward)
     */
    void setFeederRightRPM(double rpm);

    /**
     * Sets the target RPM for the shooter using onboard velocity PID.
     *
     * @param rpm target RPM (positive = shoot direction)
     */
    void setShooterRPM(double rpm);

    /** @return left feeder motor velocity in RPM */
    double getFeederLeftRPM();

    /** @return right feeder motor velocity in RPM */
    double getFeederRightRPM();

    /** @return shooter motor velocity in RPM */
    double getShooterRPM();

    /** @return left feeder motor current in amps */
    double getFeederLeftAmps();

    /** @return right feeder motor current in amps */
    double getFeederRightAmps();

    /** @return shooter motor current in amps */
    double getShooterAmps();

    /**
     * Resets/reconfigures the left feeder onboard velocity PID.
     * Call this when starting a command to pick up live-tuned values.
     */
    void resetFeederLeftPID(double kV, double kP, double kI, double kD);

    /**
     * Resets/reconfigures the right feeder onboard velocity PID.
     * Call this when starting a command to pick up live-tuned values.
     */
    void resetFeederRightPID(double kV, double kP, double kI, double kD);

    /**
     * Resets/reconfigures the shooter onboard velocity PID.
     * Call this when starting a command to pick up live-tuned values.
     */
    void resetShooterPID(double kV, double kP, double kI, double kD);

    /** Stops all three motors */
    default void stopAll() {
        setFeederLeftRPM(0);
        setFeederRightRPM(0);
        setShooterRPM(0);
    }
}

package frc.robot.subsystems.launcher;

/**
 * Interface for the launcher hardware: 2 feeder motors + 1 shooter motor + 1 shooter intake motor.
 * <p>
 * All motors use onboard closed-loop velocity control (SparkMax PID).
 * Feeders spin inward to pull balls in; shooter flings them out.
 * Shooter intake sits on the shooter side and feeds into the shooter during shots.
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
     * Sets the target RPM for the shooter intake using onboard velocity PID.
     *
     * @param rpm target RPM (positive = same direction as shooter)
     */
    void setShooterIntakeRPM(double rpm);

    /** @return shooter intake motor velocity in RPM */
    double getShooterIntakeRPM();

    /** @return shooter intake motor current in amps */
    double getShooterIntakeAmps();

    /**
     * Resets/reconfigures the shooter intake onboard velocity PID.
     */
    void resetShooterIntakePID(double kV, double kP, double kI, double kD);

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

    /** Stops all motors */
    default void stopAll() {
        setFeederLeftRPM(0);
        setFeederRightRPM(0);
        setShooterRPM(0);
        setShooterIntakeRPM(0);
    }
}

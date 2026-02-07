package frc.robot.subsystems.launcher;

/**
 * Interface for the hardware of a launcher subsystem with 4 NEO motors:
 * intake, agitator, lower wheel, and upper wheel.
 */
public interface LauncherHardware {

    /** Applies voltage to the intake motor */
    void applyIntakeVolts(double volts);

    /** Applies voltage to the agitator motor */
    void applyAgitatorVolts(double volts);

    /** Applies voltage to the lower wheel motor */
    void applyLowerWheelVolts(double volts);

    /** Applies voltage to the upper wheel motor */
    void applyUpperWheelVolts(double volts);

    /** @return intake motor velocity in RPM */
    double getIntakeRPM();

    /** @return agitator motor velocity in RPM */
    double getAgitatorRPM();

    /** @return lower wheel motor velocity in RPM */
    double getLowerWheelRPM();

    /** @return upper wheel motor velocity in RPM */
    double getUpperWheelRPM();

    /** @return intake motor current in amps */
    double getIntakeAmps();

    /** @return agitator motor current in amps */
    double getAgitatorAmps();

    /** @return lower wheel motor current in amps */
    double getLowerWheelAmps();

    /** @return upper wheel motor current in amps */
    double getUpperWheelAmps();

    /** Stops all motors */
    default void stopAll() {
        applyIntakeVolts(0);
        applyAgitatorVolts(0);
        applyLowerWheelVolts(0);
        applyUpperWheelVolts(0);
    }
}

package frc.robot.subsystems.hubbardshooter;

/**
 * Interface for the HubbardShooter hardware.
 * <p>
 * Two NEO motors (CAN 51, 52) with 12:1 gear reduction (3:1 * 4:1).
 * Open-loop voltage control for intake, outtake, and shooting modes.
 */
public interface HubbardShooterHardware {

    /**
     * Applies voltage to both motors.
     *
     * @param volts1 voltage for motor 1 (-12 to +12)
     * @param volts2 voltage for motor 2 (-12 to +12)
     */
    void applyVoltage(double volts1, double volts2);

    /** @return the current velocity of motor 1 in RPM */
    double getMotor1RPM();

    /** @return the current velocity of motor 2 in RPM */
    double getMotor2RPM();

    /** @return the output current of motor 1 in amps */
    double getMotor1Amps();

    /** @return the output current of motor 2 in amps */
    double getMotor2Amps();

    /** Stops both motors */
    default void stop() {
        applyVoltage(0, 0);
    }
}

package frc.robot.subsystems.hubberdshooter;

/**
 * Interface for the HubberdShooter hardware.
 * <p>
 * The HubberdShooter uses two motors that can operate in
 * voltage mode (intake/outtake) or velocity mode (shooting).
 */
public interface HubberdShooterHardware {

    /**
     * Applies voltage to both motors.
     * <p>
     * <ul>
     *   <li>Intake: both motors same positive voltage (pull fuel in)</li>
     *   <li>Outtake: both motors same negative voltage (push fuel out)</li>
     *   <li>Shooting: motors counter-rotate (one positive, one negative)</li>
     * </ul>
     *
     * @param volts1 voltage for motor 1 (-12 to +12)
     * @param volts2 voltage for motor 2 (-12 to +12)
     */
    void applyVoltage(double volts1, double volts2);

    /**
     * @return the current velocity of motor 1 in RPM
     */
    double getMotor1VelocityRPM();

    /**
     * @return the current velocity of motor 2 in RPM
     */
    double getMotor2VelocityRPM();

    /**
     * @return the output current of motor 1 in amps
     */
    double getMotor1Amps();

    /**
     * @return the output current of motor 2 in amps
     */
    double getMotor2Amps();

    /**
     * Stops both motors.
     */
    default void stop() {
        applyVoltage(0, 0);
    }
}

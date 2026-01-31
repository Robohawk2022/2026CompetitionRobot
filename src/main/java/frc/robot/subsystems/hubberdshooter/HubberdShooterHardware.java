package frc.robot.subsystems.hubberdshooter;

/**
 * Interface for the HubberdShooter hardware.
 * <p>
 * The HubberdShooter uses two Falcon 500 motors that can operate in
 * voltage mode (intake/outtake) or velocity mode (shooting).
 */
public interface HubberdShooterHardware {

    /**
     * Applies voltage to both motors (for intake/outtake modes).
     * <p>
     * In intake mode, both motors spin the same direction.
     * In outtake mode, both motors spin the same direction (reversed from intake).
     *
     * @param volts1 voltage for motor 1 (-12 to +12)
     * @param volts2 voltage for motor 2 (-12 to +12)
     */
    void applyVoltage(double volts1, double volts2);

    /**
     * Sets the target velocities for both motors (for shooting mode).
     * <p>
     * In shooting mode, motors counter-rotate to propel the fuel.
     *
     * @param rps1 target velocity for motor 1 in revolutions per second
     * @param rps2 target velocity for motor 2 in revolutions per second
     */
    void setVelocity(double rps1, double rps2);

    /**
     * Reconfigures the velocity PID controller with new gains.
     * <p>
     * Call this when starting a velocity control command to ensure gains are up-to-date.
     *
     * @param kV feedforward gain (volts per rev/sec)
     * @param kP proportional gain
     * @param kS static friction compensation (volts)
     */
    void configurePid(double kV, double kP, double kS);

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

package frc.robot.subsystems.intakefront;

/**
 * Interface for the front intake hardware.
 * <p>
 * The front intake is a simple motor that spins to suck in game pieces.
 */
public interface IntakeFrontHardware {

    /**
     * Applies voltage to the intake motor.
     *
     * @param volts the voltage to apply (-12 to 12)
     */
    void applyVolts(double volts);

    /**
     * @return the current motor velocity in RPM
     */
    double getVelocityRPM();

    /**
     * @return the motor output current in amps
     */
    double getMotorAmps();

    /**
     * Stops the motor.
     */
    void stop();
}

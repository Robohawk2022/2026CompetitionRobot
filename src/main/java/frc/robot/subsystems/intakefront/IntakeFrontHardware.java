package frc.robot.subsystems.intakefront;

/**
 * Interface for the front intake hardware.
 * <p>
 * The front intake uses closed-loop velocity control to spin
 * and suck in game pieces.
 */
public interface IntakeFrontHardware {

    /**
     * Sets the target speed for the intake motor using closed-loop velocity control.
     *
     * @param speedRPS the target speed in revolutions per second (positive = intake direction)
     */
    void setSpeed(double speedRPS);

    /**
     * Resets/reconfigures the velocity PID controller with new gains.
     * <p>
     * Call this when starting a command to ensure gains are up-to-date.
     *
     * @param kV feedforward gain (volts per rev/sec)
     * @param kP proportional gain
     */
    void resetPid(double kV, double kP);

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
    default void stop() {
        setSpeed(0);
    }
}

package frc.robot.subsystems.shooter;

/**
 * <p>This is the interface for the shooter hardware. The implementation will
 * talk to the motors to drive them. Separating this code out through an
 * interface makes it easier to build simulation for testing commands.</p>
 *
 * <p>We are going to be driving the motors in one of two modes: either by
 * telling them how many volts to apply ("open loop") or telling them the
 * desired velocity ("closed loop").</p>
 *
 * <p>Helpful resources:</p>
 * <ul>
 *     <li></li>
 *     <li><a href="https://docs.revrobotics.com/revlib/spark/closed-loop">Intro to Closed Loop (REV)</a>></li>
 *     <li><a href="https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started">Getting started with closed loop (REV)</a>></li>
 *     <li><a href="https://docs.revrobotics.com/revlib/spark/closed-loop/velocity-control-mode">Velocity closed loop (REV)</a>></li>
 * </ul>
 */
public interface ShooterHardware {

    /**
     * @return the current speed of the motor in revolutions per second
     */
    double getMotorRevolutionsPerSecond();

    /**
     * @return the output current reported by the motor
     */
    double getMotorAmps();

    /**
     * Sets the idle behavior of the motor
     *
     * @param brakeMode true if the motor should brake; false if it should coast
     */
    void setBrakeMode(boolean brakeMode);

    /**
     * Resets the tuning parameters for closed loop mode on the SparkMax
     *
     * @param p the P term for feedback
     * @param d the D term for feedback
     * @param v the velocity feedforward value
     */
    void resetPid(double p, double d, double v);

    /**
     * Drive the wheels at a specific speed. This should use the SparkMax
     * kVelocity closed loop mode to achieve the desired velocity in motor
     * revolutions per second. Either this or {@link #applyVoltage(double)}
     * will be called every cycle to drive the motors.
     *
     * @param revolutionsPerSecond desired motor RPS
     */
    void applySpeed(double revolutionsPerSecond);

    /**
     * Drive the wheels at a specific voltage. This should use the SparkMax
     * methods to apply motor voltage. Either this or
     * {@link #applySpeed(double)}} will be called every cycle to drive the
     * motors.
     *
     * @param voltage desired motor voltage (-12 to 12)
     */
    void applyVoltage(double voltage);

}

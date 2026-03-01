package frc.robot.subsystems.shooter;

/**
 * Interface for the hardware of a shooter subsystem (single motor).
 */
public interface ShooterHardware {

    /** @return current shooter velocity in RPM */
    double getShooterVelocity();

    /** Reset PID parameters from config */
    void resetPid();

    /** @return motor output current in amps */
    double getShooterAmps();

    /** Run the shooter motor at desired RPM (0.0 means coast) */
    void applyRpm(double shooterRpm);

}

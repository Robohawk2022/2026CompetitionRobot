package frc.robot.subsystems.ballpath;

/**
 * Interface for the hardware of a ball-path subsystem (intake, feeder, agitator).
 */
public interface BallPathHardware {

    /** @return current intake velocity in RPM */
    double getIntakeVelocity();

    /** @return current feeder velocity in RPM */
    double getFeederVelocity();

    /** @return current agitator velocity in RPM */
    double getAgitatorVelocity();

    /** Reset PID parameters from config */
    void resetPid();

    /** Run the motors at desired RPM (0.0 means coast) */
    void applyRpm(double intakeRpm, double feederRpm, double agitatorRpm);

}

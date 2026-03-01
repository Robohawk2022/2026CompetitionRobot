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

    /** @return intake motor output current in amps */
    double getIntakeAmps();

    /** @return feeder motor output current in amps */
    double getFeederAmps();

    /** @return agitator motor output current in amps */
    double getAgitatorAmps();

    /** Reset PID parameters from config */
    void resetPid();

    /** Run the motors at desired RPM (0.0 means coast) */
    void applyRpm(double intakeRpm, double feederRpm, double agitatorRpm);

}

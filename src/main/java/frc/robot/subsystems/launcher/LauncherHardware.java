package frc.robot.subsystems.launcher;

public interface LauncherHardware {

    /** Get velocity of the different motors */
    double getIntakeVelocity();
    double getFeederVelocity();
    double getAgitatorVelocity();
    double getShooterVelocity();

    /** Reset PID parameters */
    void resetPid();

    /** Run the motors at desired RPM (0.0 means coast) */
    void applyRpm(double intakeRpm, double feederRpm, double agitatorRpm, double shooterRpm);

}

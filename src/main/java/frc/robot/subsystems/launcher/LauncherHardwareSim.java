package frc.robot.subsystems.launcher;

/**
 * Implementation of {@link LauncherHardware} for running in simulation
 */
public class LauncherHardwareSim implements LauncherHardware {

    double intakeRpm;
    double feederRpm;
    double agitatorRpm;
    double shooterRpm;

    @Override
    public double getIntakeVelocity() {
        return intakeRpm;
    }

    @Override
    public double getFeederVelocity() {
        return feederRpm;
    }

    @Override
    public double getAgitatorVelocity() {
        return agitatorRpm;
    }

    @Override
    public double getShooterVelocity() {
        return shooterRpm;
    }

    @Override
    public void resetPid() {

    }

    @Override
    public void applyRpm(double intakeRpm, double feederRpm, double agitatorRpm, double shooterRpm) {
        this.intakeRpm = intakeRpm;
        this.feederRpm = feederRpm;
        this.agitatorRpm = agitatorRpm;
        this.shooterRpm = shooterRpm;
    }
}

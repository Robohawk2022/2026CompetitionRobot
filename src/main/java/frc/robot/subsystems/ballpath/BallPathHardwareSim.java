package frc.robot.subsystems.ballpath;

/**
 * Implementation of {@link BallPathHardware} for running in simulation.
 */
public class BallPathHardwareSim implements BallPathHardware {

    double intakeRpm;
    double feederRpm;
    double agitatorRpm;

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
    public void resetPid() {
    }

    @Override
    public void applyRpm(double intakeRpm, double feederRpm, double agitatorRpm) {
        this.intakeRpm = intakeRpm;
        this.feederRpm = feederRpm;
        this.agitatorRpm = agitatorRpm;
    }
}

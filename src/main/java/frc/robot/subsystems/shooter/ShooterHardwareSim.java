package frc.robot.subsystems.shooter;

/**
 * Implementation of {@link ShooterHardware} for running in simulation.
 */
public class ShooterHardwareSim implements ShooterHardware {

    double shooterRpm;

    @Override
    public double getShooterVelocity() {
        return shooterRpm;
    }

    @Override
    public double getShooterAmps() {
        return 0.0;
    }

    @Override
    public void resetPid() {
    }

    @Override
    public void applyRpm(double shooterRpm) {
        this.shooterRpm = shooterRpm;
    }
}

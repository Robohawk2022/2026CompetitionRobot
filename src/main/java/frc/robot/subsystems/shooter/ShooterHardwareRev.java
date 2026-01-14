package frc.robot.subsystems.shooter;

/**
 * Implementation of the hardware interface on top of real REV motors.
 */
public class ShooterHardwareRev implements ShooterHardware {

    @Override
    public double getMotorRevolutionsPerSecond() {
        throw new UnsupportedOperationException("TODO implement me");
    }

    @Override
    public double getMotorAmps() {
        throw new UnsupportedOperationException("TODO implement me");
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        throw new UnsupportedOperationException("TODO implement me");
    }

    @Override
    public void resetPid(double p, double d, double v) {
        throw new UnsupportedOperationException("TODO implement me");
    }

    @Override
    public void applySpeed(double revolutionsPerSecond) {
        throw new UnsupportedOperationException("TODO implement me");
    }

    @Override
    public void applyVolts(double volts) {
        throw new UnsupportedOperationException("TODO implement me");
    }
}

package frc.robot.subsystems.launcher;

/**
 * Implements {@link LauncherHardware} for simulation.
 * <p>
 * Simulates three NEO motors with first-order dynamics.
 */
public class LauncherHardwareSim implements LauncherHardware {

    private static final double MAX_RPM = 5676; // NEO free speed
    private static final double TIME_CONSTANT = 0.4; // first-order response

    private double feederLeftRPM, feederLeftTargetRPM;
    private double feederRightRPM, feederRightTargetRPM;
    private double shooterRPM, shooterTargetRPM;
    private double flapperRPM, flapperTargetRPM;

    private double updateMotor(double current, double target) {
        double clamped = Math.max(-MAX_RPM, Math.min(MAX_RPM, target));
        return current + (clamped - current) * TIME_CONSTANT;
    }

    private double simulateAmps(double current, double target) {
        double baseAmps = Math.abs(current / MAX_RPM) * 10.0;
        double accelAmps = Math.abs(target - current) / MAX_RPM * 20.0;
        return baseAmps + accelAmps;
    }

    @Override
    public void setFeederLeftRPM(double rpm) {
        feederLeftTargetRPM = rpm;
        feederLeftRPM = updateMotor(feederLeftRPM, feederLeftTargetRPM);
    }

    @Override
    public void setFeederRightRPM(double rpm) {
        feederRightTargetRPM = rpm;
        feederRightRPM = updateMotor(feederRightRPM, feederRightTargetRPM);
    }

    @Override
    public void setShooterRPM(double rpm) {
        shooterTargetRPM = rpm;
        shooterRPM = updateMotor(shooterRPM, shooterTargetRPM);
    }

    @Override
    public void setFlapperRPM(double rpm) {
        flapperTargetRPM = rpm;
        flapperRPM = updateMotor(flapperRPM, flapperTargetRPM);
    }

    @Override public void resetFeederLeftPID(double kV, double kP, double kI, double kD) { }
    @Override public void resetFeederRightPID(double kV, double kP, double kI, double kD) { }
    @Override public void resetShooterPID(double kV, double kP, double kI, double kD) { }
    @Override public void resetFlapperPID(double kV, double kP, double kI, double kD) { }

    @Override public double getFeederLeftRPM() { return feederLeftRPM; }
    @Override public double getFeederRightRPM() { return feederRightRPM; }
    @Override public double getShooterRPM() { return shooterRPM; }

    @Override public double getFlapperRPM() { return flapperRPM; }

    @Override public double getFeederLeftAmps() { return simulateAmps(feederLeftRPM, feederLeftTargetRPM); }
    @Override public double getFeederRightAmps() { return simulateAmps(feederRightRPM, feederRightTargetRPM); }
    @Override public double getShooterAmps() { return simulateAmps(shooterRPM, shooterTargetRPM); }
    @Override public double getFlapperAmps() { return simulateAmps(flapperRPM, flapperTargetRPM); }

    @Override
    public void stopAll() {
        feederLeftTargetRPM = 0; feederLeftRPM *= 0.1;
        feederRightTargetRPM = 0; feederRightRPM *= 0.1;
        shooterTargetRPM = 0; shooterRPM *= 0.1;
        flapperTargetRPM = 0; flapperRPM *= 0.1;
    }
}

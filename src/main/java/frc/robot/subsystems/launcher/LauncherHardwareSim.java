package frc.robot.subsystems.launcher;

/**
 * Implements {@link LauncherHardware} for simulation.
 * <p>
 * Simulates three NEO motors with first-order dynamics.
 * Wheels accept RPM targets (simulating onboard PID).
 */
public class LauncherHardwareSim implements LauncherHardware {

    private static final double MAX_RPM = 5676; // NEO free speed
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double TIME_CONSTANT = 0.4; // first-order response

    // simulation state (in RPM)
    private double agitatorRPM, agitatorTargetRPM;
    private double lowerWheelRPM, lowerWheelTargetRPM;
    private double upperWheelRPM, upperWheelTargetRPM;

    private double updateMotor(double current, double target) {
        double clamped = Math.max(-MAX_RPM, Math.min(MAX_RPM, target));
        return current + (clamped - current) * TIME_CONSTANT;
    }

    private double voltsToRPM(double volts) {
        return (volts / NOMINAL_VOLTAGE) * MAX_RPM;
    }

    private double simulateAmps(double current, double target) {
        double baseAmps = Math.abs(current / MAX_RPM) * 10.0;
        double accelAmps = Math.abs(target - current) / MAX_RPM * 20.0;
        return baseAmps + accelAmps;
    }

    @Override
    public void setLowerWheelRPM(double rpm) {
        lowerWheelTargetRPM = rpm;
        lowerWheelRPM = updateMotor(lowerWheelRPM, lowerWheelTargetRPM);
    }

    @Override
    public void setUpperWheelRPM(double rpm) {
        upperWheelTargetRPM = rpm;
        upperWheelRPM = updateMotor(upperWheelRPM, upperWheelTargetRPM);
    }

    @Override
    public void applyAgitatorVolts(double volts) {
        agitatorTargetRPM = voltsToRPM(volts);
        agitatorRPM = updateMotor(agitatorRPM, agitatorTargetRPM);
    }

    @Override
    public void resetPID(double kV, double kP, double kD) {
        // no-op in simulation
    }

    @Override public double getAgitatorRPM() { return agitatorRPM; }
    @Override public double getLowerWheelRPM() { return lowerWheelRPM; }
    @Override public double getUpperWheelRPM() { return upperWheelRPM; }

    @Override public double getAgitatorAmps() { return simulateAmps(agitatorRPM, agitatorTargetRPM); }
    @Override public double getLowerWheelAmps() { return simulateAmps(lowerWheelRPM, lowerWheelTargetRPM); }
    @Override public double getUpperWheelAmps() { return simulateAmps(upperWheelRPM, upperWheelTargetRPM); }

    @Override
    public void stopAll() {
        agitatorTargetRPM = 0; agitatorRPM *= 0.1;
        lowerWheelTargetRPM = 0; lowerWheelRPM *= 0.1;
        upperWheelTargetRPM = 0; upperWheelRPM *= 0.1;
    }
}

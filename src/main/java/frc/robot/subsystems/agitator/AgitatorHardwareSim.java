package frc.robot.subsystems.agitator;

/**
 * Implements {@link AgitatorHardware} for simulation.
 */
public class AgitatorHardwareSim implements AgitatorHardware {

    private static final double MAX_RPM = 5676;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double TIME_CONSTANT = 0.4;

    private double rpm, targetRPM;

    @Override
    public void applyVolts(double volts) {
        targetRPM = (volts / NOMINAL_VOLTAGE) * MAX_RPM;
        rpm = rpm + (targetRPM - rpm) * TIME_CONSTANT;
    }

    @Override
    public double getRPM() {
        return rpm;
    }

    @Override
    public double getAmps() {
        double baseAmps = Math.abs(rpm / MAX_RPM) * 10.0;
        double accelAmps = Math.abs(targetRPM - rpm) / MAX_RPM * 20.0;
        return baseAmps + accelAmps;
    }

    @Override
    public void stop() {
        targetRPM = 0;
        rpm *= 0.1;
    }
}

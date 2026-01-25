package frc.robot.subsystems.intakefront;

import edu.wpi.first.math.MathUtil;

/**
 * Implements {@link IntakeFrontHardware} for simulation.
 */
public class IntakeFrontHardwareSim implements IntakeFrontHardware {

    private static final double MAX_VOLTAGE = 12.0;
    private static final double MAX_RPM = 5700; // NEO free speed
    private static final double VOLTAGE_TO_RPM = MAX_RPM / MAX_VOLTAGE;

    private double appliedVolts = 0;
    private double velocityRPM = 0;

    @Override
    public void applyVolts(double volts) {
        // Clamp voltage for safety (matches real hardware behavior)
        appliedVolts = MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE);
        // Simple simulation: velocity proportional to voltage
        velocityRPM = appliedVolts * VOLTAGE_TO_RPM;
    }

    @Override
    public double getVelocityRPM() {
        return velocityRPM;
    }

    @Override
    public double getMotorAmps() {
        // Simulate current draw based on voltage (~24A at 12V)
        return Math.abs(appliedVolts) * 2.0;
    }
}

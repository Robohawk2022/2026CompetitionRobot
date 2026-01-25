package frc.robot.subsystems.intakefront;

/**
 * Implements {@link IntakeFrontHardware} for simulation.
 */
public class IntakeFrontHardwareSim implements IntakeFrontHardware {

    private double appliedVolts = 0;
    private double velocityRPM = 0;

    // Simulated motor constants
    private static final double MAX_RPM = 5700; // NEO free speed
    private static final double VOLTAGE_TO_RPM = MAX_RPM / 12.0;

    @Override
    public void applyVolts(double volts) {
        appliedVolts = volts;
        // Simple simulation: velocity proportional to voltage
        velocityRPM = volts * VOLTAGE_TO_RPM;
    }

    @Override
    public double getVelocityRPM() {
        return velocityRPM;
    }

    @Override
    public double getMotorAmps() {
        // Simulate current draw based on voltage
        return Math.abs(appliedVolts) * 2.0; // ~24A at 12V
    }

    @Override
    public void stop() {
        appliedVolts = 0;
        velocityRPM = 0;
    }
}

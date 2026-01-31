package frc.robot.subsystems.hubberdshooter;

/**
 * Implements {@link HubberdShooterHardware} for simulation.
 * <p>
 * Simulates two flywheel motors with first-order dynamics.
 */
public class HubberdShooterHardwareSim implements HubberdShooterHardware {

    private static final double MAX_RPM = 6380; // Falcon 500 free speed
    private static final double MAX_RPS = MAX_RPM / 60.0;
    private static final double NOMINAL_VOLTAGE = 12.0;

    // simulation state
    private double motor1SpeedRPS = 0;
    private double motor2SpeedRPS = 0;
    private double targetSpeed1RPS = 0;
    private double targetSpeed2RPS = 0;
    private boolean velocityMode = false;

    // PID gains (for velocity mode simulation)
    private double kV = 0.12;
    private double kP = 0.1;
    private double kS = 0.0;

    @Override
    public void applyVoltage(double volts1, double volts2) {
        velocityMode = false;

        // Convert voltage to approximate target speed (V/12 * maxSpeed)
        targetSpeed1RPS = (volts1 / NOMINAL_VOLTAGE) * MAX_RPS;
        targetSpeed2RPS = (volts2 / NOMINAL_VOLTAGE) * MAX_RPS;

        updateSimulation();
    }

    @Override
    public void setVelocity(double rps1, double rps2) {
        velocityMode = true;
        targetSpeed1RPS = rps1;
        targetSpeed2RPS = rps2;

        updateSimulation();
    }

    /**
     * Updates the simulation state with first-order dynamics.
     * Called after setting voltage or velocity targets.
     */
    private void updateSimulation() {
        // Simple first-order response with ~50ms time constant
        double timeConstant = 0.4; // ~50ms at 20ms update rate

        if (velocityMode) {
            // In velocity mode, simulate closed-loop tracking
            double error1 = targetSpeed1RPS - motor1SpeedRPS;
            double error2 = targetSpeed2RPS - motor2SpeedRPS;
            motor1SpeedRPS += error1 * timeConstant;
            motor2SpeedRPS += error2 * timeConstant;
        } else {
            // In voltage mode, simulate open-loop response
            double error1 = targetSpeed1RPS - motor1SpeedRPS;
            double error2 = targetSpeed2RPS - motor2SpeedRPS;
            motor1SpeedRPS += error1 * timeConstant;
            motor2SpeedRPS += error2 * timeConstant;
        }

        // Clamp to motor limits
        motor1SpeedRPS = Math.max(-MAX_RPS, Math.min(MAX_RPS, motor1SpeedRPS));
        motor2SpeedRPS = Math.max(-MAX_RPS, Math.min(MAX_RPS, motor2SpeedRPS));
    }

    @Override
    public void configurePid(double kV, double kP, double kS) {
        this.kV = kV;
        this.kP = kP;
        this.kS = kS;
    }

    @Override
    public double getMotor1VelocityRPM() {
        return motor1SpeedRPS * 60.0;
    }

    @Override
    public double getMotor2VelocityRPM() {
        return motor2SpeedRPS * 60.0;
    }

    @Override
    public double getMotor1Amps() {
        // Simulate current draw based on speed and acceleration
        double baseAmps = Math.abs(motor1SpeedRPS / MAX_RPS) * 10.0;
        double accelAmps = Math.abs(targetSpeed1RPS - motor1SpeedRPS) * 2.0;
        return baseAmps + accelAmps;
    }

    @Override
    public double getMotor2Amps() {
        double baseAmps = Math.abs(motor2SpeedRPS / MAX_RPS) * 10.0;
        double accelAmps = Math.abs(targetSpeed2RPS - motor2SpeedRPS) * 2.0;
        return baseAmps + accelAmps;
    }

    @Override
    public void stop() {
        velocityMode = false;
        targetSpeed1RPS = 0;
        targetSpeed2RPS = 0;
        // Simulate quick stop (brake-like behavior)
        motor1SpeedRPS *= 0.1;
        motor2SpeedRPS *= 0.1;
    }
}

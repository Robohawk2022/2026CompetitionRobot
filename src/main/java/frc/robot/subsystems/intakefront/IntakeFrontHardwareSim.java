package frc.robot.subsystems.intakefront;

import frc.robot.util.Util;

/**
 * Implements {@link IntakeFrontHardware} for simulation with closed-loop velocity control.
 */
public class IntakeFrontHardwareSim implements IntakeFrontHardware {

    private static final double MAX_RPM = 5700; // NEO free speed
    private static final double MAX_RPS = MAX_RPM / 60.0;

    private double targetSpeedRPS = 0;
    private double currentSpeedRPS = 0;
    private double kV = 0.12;
    private double kP = 0.1;

    @Override
    public void setSpeed(double speedRPS) {
        targetSpeedRPS = speedRPS;
        // Simulate closed-loop: gradually approach target
        // Simple first-order response with ~50ms time constant
        double error = targetSpeedRPS - currentSpeedRPS;
        currentSpeedRPS += error * 0.4; // ~50ms time constant at 20ms update rate
        // Clamp to motor limits
        currentSpeedRPS = Math.max(-MAX_RPS, Math.min(MAX_RPS, currentSpeedRPS));
    }

    @Override
    public void resetPid(double kV, double kP) {
        this.kV = kV;
        this.kP = kP;
    }

    @Override
    public double getVelocityRPM() {
        return currentSpeedRPS * 60.0;
    }

    @Override
    public double getMotorAmps() {
        // Simulate current draw: higher when accelerating or under load
        double baseAmps = Math.abs(currentSpeedRPS / MAX_RPS) * 10.0;
        double accelAmps = Math.abs(targetSpeedRPS - currentSpeedRPS) * 2.0;
        return baseAmps + accelAmps;
    }

    @Override
    public void stop() {
        targetSpeedRPS = 0;
        // Simulate quick stop (brake mode)
        currentSpeedRPS *= 0.1;
    }
}

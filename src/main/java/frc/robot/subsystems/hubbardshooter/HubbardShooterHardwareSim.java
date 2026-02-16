package frc.robot.subsystems.hubbardshooter;

import frc.robot.util.Util;

/**
 * Implements {@link HubbardShooterHardware} for simulation.
 * <p>
 * Simulates two NEO motors with first-order dynamics.
 * 12:1 gear reduction (3:1 * 4:1) — output RPM is motor RPM / 12.
 */
public class HubbardShooterHardwareSim implements HubbardShooterHardware {

    private static final double MAX_RPM = 5676; // NEO free speed
    private static final double TIME_CONSTANT = 0.4;

    private double motor1RPM, motor1Target;
    private double motor2RPM, motor2Target;

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
    public void applyVoltage(double volts1, double volts2) {
        // convert voltage to target RPM (proportional to voltage)
        motor1Target = (volts1 / Util.MAX_VOLTS) * MAX_RPM;
        motor2Target = (volts2 / Util.MAX_VOLTS) * MAX_RPM;
        motor1RPM = updateMotor(motor1RPM, motor1Target);
        motor2RPM = updateMotor(motor2RPM, motor2Target);
    }

    @Override
    public double getMotor1RPM() {
        return motor1RPM;
    }

    @Override
    public double getMotor2RPM() {
        return motor2RPM;
    }

    @Override
    public double getMotor1Amps() {
        return simulateAmps(motor1RPM, motor1Target);
    }

    @Override
    public double getMotor2Amps() {
        return simulateAmps(motor2RPM, motor2Target);
    }

    @Override
    public void stop() {
        motor1Target = 0; motor1RPM *= 0.1;
        motor2Target = 0; motor2RPM *= 0.1;
    }
}

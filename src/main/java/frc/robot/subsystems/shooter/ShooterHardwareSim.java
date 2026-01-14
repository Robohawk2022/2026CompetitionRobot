package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Util;

/**
 * Super-simple simulation of a shooter. This lets us use the simulator to
 * test logic that depends on the shooter, without having to have access to
 * the physical robot.
 */
public class ShooterHardwareSim implements ShooterHardware {

    final PIDController pid;
    boolean brake = false;
    double velocity = 0.0;
    double position = 0.0;
    double v = 0.0;

    public ShooterHardwareSim() {
        pid = new PIDController(0.0, 0.0, 0.0);
    }

    @Override
    public double getMotorAmps() {
        // we're not going to simulate amps
        return 0;
    }

    @Override
    public double getMotorRevolutionsPerSecond() {
        return velocity;
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        brake = brakeMode;
    }

    @Override
    public void resetPid(double p, double d, double v) {
        this.v = v;
        pid.setPID(p, 0.0, d);
        pid.reset();
    }

    @Override
    public void applyVolts(double volts) {
        if (volts == 0.0) {
            velocity = calculateIdle(velocity);
        } else {
            velocity = calculateSpeed(volts);
        }
        position += velocity * Util.DT;
    }

    @Override
    public void applySpeed(double rps) {

        // this is how you do feedforward and feedback in software. the motor
        // controller is essentially doing these same calculations in
        // hardware.
        double ff = v * rps;
        double fb = pid.calculate(getMotorRevolutionsPerSecond(), rps);
        applyVolts(Util.clampVolts(ff + fb));
    }

    /*
     * Arbitrary "coasting" calculation. If we're braking, we assume we stop
     * immediately. Otherwise, we assume speed drops off by 5% every cycle
     * until it gets close to a minimum speed (this could be replaced with
     * a real simulation if you want).
     */
    private double calculateIdle(double currentVelocity) {
        if (brake) {
            return 0.0;
        }
        return Math.min(2.0, currentVelocity * 0.95);
    }

    /*
     * Arbitrary output speed calculation - we'll assume we instantly attain
     * velocity proportional to input voltage, up to a maximum of 100 rps
     * (this could be replaced with a real simulation if you want)
     */
    private double calculateSpeed(double currentVoltage) {
        return 100.0 * (currentVoltage / Util.MAX_VOLTS);
    }
}
package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleSupplier;

/**
 * Calculates the rate of change for an underlying value
 */
public class RateCalculator {

    final DoubleSupplier supplier;
    double lastTimestamp;
    double lastValue;

    /**
     * Creates a {@link RateCalculator}
     * @param supplier supplier for the underlying value
     */
    public RateCalculator(DoubleSupplier supplier) {
        this.supplier = supplier;
        this.lastTimestamp = Double.NaN;
        this.lastValue = Double.NaN;
    }

    /**
     * @return the rate of change of the underlying value since the
     * last call to this method (the first time this is called it will
     * return {@link Double#NaN})
     */
    public double calculate() {

        double now = Timer.getFPGATimestamp();
        double currentValue = supplier.getAsDouble();
        double currentRate = Double.NaN;

        if (Double.isFinite(lastTimestamp)) {
            currentRate = (currentValue - lastValue) / (now - lastTimestamp);
        }

        lastTimestamp = now;
        lastValue = currentValue;
        return currentRate;
    }
}

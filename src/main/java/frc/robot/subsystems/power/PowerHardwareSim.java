package frc.robot.subsystems.power;

/**
 * Implements {@link PowerHardware} for simulation with reasonable defaults.
 */
public class PowerHardwareSim implements PowerHardware {

    static final int CHANNEL_COUNT = 24;

    @Override
    public double getChannelCurrent(int channel) {
        return 0.0;
    }

    @Override
    public double getTotalCurrent() {
        return 0.0;
    }

    @Override
    public double getVoltage() {
        return 12.0;
    }

    @Override
    public double getTotalPower() {
        return 0.0;
    }

    @Override
    public double getTemperature() {
        return 25.0;
    }

    @Override
    public int getChannelCount() {
        return CHANNEL_COUNT;
    }
}

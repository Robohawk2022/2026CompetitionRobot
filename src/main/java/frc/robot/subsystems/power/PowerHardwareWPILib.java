package frc.robot.subsystems.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import java.util.Objects;

/**
 * Implements {@link PowerHardware} using WPILib's PowerDistribution class.
 */
public class PowerHardwareWPILib implements PowerHardware {

    final PowerDistribution pdh;

    /**
     * Creates a {@link PowerHardwareWPILib}.
     *
     * @param canId the CAN ID of the power distribution module
     * @param moduleType the module type (REV or CTRE)
     */
    public PowerHardwareWPILib(int canId, ModuleType moduleType) {
        this.pdh = Objects.requireNonNull(new PowerDistribution(canId, moduleType));
    }

    @Override
    public double getChannelCurrent(int channel) {
        return pdh.getCurrent(channel);
    }

    @Override
    public double getTotalCurrent() {
        return pdh.getTotalCurrent();
    }

    @Override
    public double getVoltage() {
        return pdh.getVoltage();
    }

    @Override
    public double getTotalPower() {
        return pdh.getTotalPower();
    }

    @Override
    public double getTemperature() {
        return pdh.getTemperature();
    }

    @Override
    public int getChannelCount() {
        return pdh.getNumChannels();
    }
}

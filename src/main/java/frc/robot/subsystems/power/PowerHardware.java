package frc.robot.subsystems.power;

/**
 * Interface for the hardware of a power monitoring subsystem.
 */
public interface PowerHardware {

    /** @return current in amps for a specific channel */
    double getChannelCurrent(int channel);

    /** @return total current draw in amps */
    double getTotalCurrent();

    /** @return battery voltage */
    double getVoltage();

    /** @return total power in watts */
    double getTotalPower();

    /** @return PDH temperature in Celsius */
    double getTemperature();

    /** @return number of channels (24 for REV PDH, 16 for CTRE PDP) */
    int getChannelCount();
}

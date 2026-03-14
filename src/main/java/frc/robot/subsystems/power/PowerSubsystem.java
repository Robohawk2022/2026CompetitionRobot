package frc.robot.subsystems.power;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;
import java.util.Objects;

import static frc.robot.Config.Power.*;

/**
 * <p>Read-only monitoring subsystem that tracks power distribution.</p>
 *
 * <p>Reads all PDH channel currents each cycle and identifies the top
 * channels by current draw. Publishes battery voltage, total current,
 * total power, temperature, and top-drawing channels to the dashboard.</p>
 */
public class PowerSubsystem extends SubsystemBase {

    static final boolean verboseLogging = true;

//region Implementation --------------------------------------------------------

    final PowerHardware hardware;
    final Map<Integer, String> channelNames;
    final double[] channelCurrents;
    final int[] topChannels;
    final double[] topCurrents;

    double currentVoltage;
    double currentTotalCurrent;
    double currentTotalPower;
    double currentTemperature;

    /**
     * Creates a {@link PowerSubsystem}.
     *
     * @param hardware the hardware interface (required)
     * @param channelNames map of channel number to mechanism name (e.g., 35 -> "Shooter")
     */
    public PowerSubsystem(PowerHardware hardware, Map<Integer, String> channelNames) {
        this.hardware = Objects.requireNonNull(hardware);
        this.channelNames = Objects.requireNonNull(channelNames);
        this.channelCurrents = new double[hardware.getChannelCount()];
        this.topChannels = new int[3];
        this.topCurrents = new double[3];

        SmartDashboard.putData(getName(), builder -> {
            builder.addDoubleProperty("Voltage", () -> currentVoltage, null);
            builder.addDoubleProperty("CurrentTotal", () -> currentTotalCurrent, null);
            builder.addDoubleProperty("PowerTotal", () -> currentTotalPower, null);
            builder.addDoubleProperty("Temperature", () -> currentTemperature, null);

            int count = (int) topChannelCount.getAsDouble();
            for (int i = 0; i < 3; i++) {
                final int idx = i;
                if (idx < count) {
                    builder.addStringProperty("TopChannel" + (idx + 1),
                            () -> getChannelLabel(topChannels[idx]), null);
                    builder.addDoubleProperty("TopChannel" + (idx + 1) + "Amps",
                            () -> topCurrents[idx], null);
                }
            }

            if (verboseLogging) {
                for (int ch = 0; ch < channelCurrents.length; ch++) {
                    final int channel = ch;
                    String label = channelNames.getOrDefault(channel, "Ch" + channel);
                    builder.addDoubleProperty("Channel/" + label,
                            () -> channelCurrents[channel], null);
                }
            }
        });
    }

    @Override
    public void periodic() {
        // read all channels
        for (int i = 0; i < channelCurrents.length; i++) {
            channelCurrents[i] = hardware.getChannelCurrent(i);
        }

        currentVoltage = hardware.getVoltage();
        currentTotalCurrent = hardware.getTotalCurrent();
        currentTotalPower = hardware.getTotalPower();
        currentTemperature = hardware.getTemperature();

        // find top channels by current draw
        findTopChannels((int) topChannelCount.getAsDouble());
    }

    /**
     * Finds the top N channels by current draw and stores them in topChannels/topCurrents.
     */
    private void findTopChannels(int count) {
        count = Math.min(count, 3);

        // reset
        for (int i = 0; i < 3; i++) {
            topChannels[i] = -1;
            topCurrents[i] = 0.0;
        }

        for (int slot = 0; slot < count; slot++) {
            double maxAmps = -1.0;
            int maxChannel = -1;

            for (int ch = 0; ch < channelCurrents.length; ch++) {
                // skip channels already in a top slot
                boolean alreadyUsed = false;
                for (int prev = 0; prev < slot; prev++) {
                    if (topChannels[prev] == ch) {
                        alreadyUsed = true;
                        break;
                    }
                }
                if (alreadyUsed) continue;

                if (channelCurrents[ch] > maxAmps) {
                    maxAmps = channelCurrents[ch];
                    maxChannel = ch;
                }
            }

            topChannels[slot] = maxChannel;
            topCurrents[slot] = maxAmps;
        }
    }

    /**
     * @return human-readable label for a channel
     */
    private String getChannelLabel(int channel) {
        if (channel < 0) return "---";
        return channelNames.getOrDefault(channel, "Ch" + channel);
    }

//endregion
}

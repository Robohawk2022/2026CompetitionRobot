package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Implements {@link LEDHardware} for simulation.
 * <p>
 * Posts the current signal name to SmartDashboard for visualization.
 */
public class LEDHardwareSim implements LEDHardware {

    private LEDSignal currentSignal = LEDSignal.OFF;

    /**
     * Creates the simulated LED hardware.
     */
    public LEDHardwareSim() {
        // Initialize dashboard display
        SmartDashboard.putString("LED/SimSignal", "OFF");
        SmartDashboard.putNumber("LED/SimBlinkinCode", 0.99);
    }

    @Override
    public void applySignal(LEDSignal signal) {
        if (signal == null) {
            signal = LEDSignal.OFF;
        }
        currentSignal = signal;

        // Post to dashboard for visualization
        SmartDashboard.putString("LED/SimSignal", signal.name());
        SmartDashboard.putNumber("LED/SimBlinkinCode", signal.getBlinkinCode());
    }

    @Override
    public LEDSignal getCurrentSignal() {
        return currentSignal;
    }
}

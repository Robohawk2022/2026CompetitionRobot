package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Implements {@link LEDHardware} for simulation.
 * <p>
 * Posts the current signal name to SmartDashboard for visualization.
 */
public class LEDHardwareSim implements LEDHardware {

    /**
     * Creates the simulated LED hardware.
     */
    public LEDHardwareSim() {
        applySignal(LEDSignal.ALL_OFF);
    }

    @Override
    public void applySignal(LEDSignal signal) {
        if (signal == null) {
            signal = LEDSignal.ALL_OFF;
        }
        SmartDashboard.putString("LED/SimSignal", signal.name());
        SmartDashboard.putNumber("LED/SimBlinkinCode", signal.getBlinkinCode());
    }
}

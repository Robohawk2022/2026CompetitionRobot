package frc.robot.subsystems.intakefront;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.IntakeFront.*;

/**
 * Subsystem for the front intake mechanism.
 * <p>
 * Spins intake motors to suck in game pieces. Direction is configurable
 * via the {@code IntakeFront/Inverted?} preference.
 */
public class IntakeFrontSubsystem extends SubsystemBase {

    //region Implementation --------------------------------------------------------

    private final IntakeFrontHardware hardware;
    private String currentMode = "idle";
    private double latestVolts = 0;

    /**
     * Creates an {@link IntakeFrontSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public IntakeFrontSubsystem(IntakeFrontHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);

        // Dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("Volts", () -> latestVolts, null);
            builder.addDoubleProperty("VelocityRPM", hardware::getVelocityRPM, null);
            builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
            builder.addBooleanProperty("Inverted?", inverted::getAsBoolean, null);
            builder.addDoubleProperty("SpeedPct", speedPercent::getAsDouble, null);
        });
    }

    @Override
    public void periodic() {
        // Nothing to do - commands handle motor control
    }

    /**
     * Applies voltage to the intake motor, respecting the inversion setting.
     *
     * @param volts the voltage to apply (positive = intake direction)
     */
    private void applyVolts(double volts) {
        // Apply inversion if configured
        if (inverted.getAsBoolean()) {
            volts = -volts;
        }
        latestVolts = Util.clampVolts(volts);
        hardware.applyVolts(latestVolts);
    }

    //endregion

    //region Command factories -----------------------------------------------------

    /**
     * @return a command that stops the intake
     */
    public Command stopCommand() {
        return runOnce(() -> {
            currentMode = "stopped";
            latestVolts = 0;
            hardware.stop();
        });
    }

    /**
     * @return a command that runs the intake to suck in game pieces
     */
    public Command intakeCommand() {
        return run(() -> {
            currentMode = "intaking";
            double pct = speedPercent.getAsDouble() / 100.0;
            applyVolts(12.0 * pct);
        }).finallyDo(() -> {
            hardware.stop();
            currentMode = "idle";
            latestVolts = 0;
        });
    }

    /**
     * @return a command that runs the intake in reverse (eject)
     */
    public Command ejectCommand() {
        return run(() -> {
            currentMode = "ejecting";
            double pct = speedPercent.getAsDouble() / 100.0;
            applyVolts(-12.0 * pct);
        }).finallyDo(() -> {
            hardware.stop();
            currentMode = "idle";
            latestVolts = 0;
        });
    }

    /**
     * @return a command that does nothing (idle)
     */
    public Command idleCommand() {
        return run(() -> {
            currentMode = "idle";
            latestVolts = 0;
            hardware.stop();
        });
    }

    //endregion

    //region Direct control methods ------------------------------------------------

    /**
     * Directly runs the intake (for use outside command system).
     */
    public void runIntake() {
        currentMode = "intaking";
        double pct = speedPercent.getAsDouble() / 100.0;
        applyVolts(12.0 * pct);
    }

    /**
     * Directly runs the intake in reverse (for use outside command system).
     */
    public void runEject() {
        currentMode = "ejecting";
        double pct = speedPercent.getAsDouble() / 100.0;
        applyVolts(-12.0 * pct);
    }

    /**
     * Directly stops the intake (for use outside command system).
     */
    public void stop() {
        currentMode = "idle";
        latestVolts = 0;
        hardware.stop();
    }

    //endregion
}

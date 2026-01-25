package frc.robot.subsystems.intakefront;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
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
     * Gets the clamped speed percentage (0-100).
     */
    private double getSpeedFactor() {
        return MathUtil.clamp(speedPercent.getAsDouble(), 0, 100) / 100.0;
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

    /**
     * Cleanup helper - stops motor and resets state.
     */
    private void cleanup() {
        hardware.stop();
        currentMode = "idle";
        latestVolts = 0;
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
            applyVolts(Util.MAX_VOLTS * getSpeedFactor());
        }).finallyDo(this::cleanup);
    }

    /**
     * @return a command that runs the intake in reverse (eject)
     */
    public Command ejectCommand() {
        return run(() -> {
            currentMode = "ejecting";
            applyVolts(-Util.MAX_VOLTS * getSpeedFactor());
        }).finallyDo(this::cleanup);
    }

    /**
     * @return a command that idles the intake (stops motor once, then does nothing)
     */
    public Command idleCommand() {
        return startRun(
            () -> {
                // Initialize: stop motor once
                currentMode = "idle";
                latestVolts = 0;
                hardware.stop();
            },
            () -> {
                // Execute: do nothing (motor stays stopped)
            }
        );
    }

    //endregion

    //region Direct control methods ------------------------------------------------
    // WARNING: These methods do not have automatic cleanup like commands.
    // If you call runIntake() or runEject(), you MUST call stop() when done,
    // or the motor will keep running indefinitely.

    /**
     * Directly runs the intake (for use outside command system).
     * <p>
     * <b>WARNING:</b> You must call {@link #stop()} when done, or the motor
     * will keep running indefinitely.
     */
    public void runIntake() {
        currentMode = "intaking";
        applyVolts(Util.MAX_VOLTS * getSpeedFactor());
    }

    /**
     * Directly runs the intake in reverse (for use outside command system).
     * <p>
     * <b>WARNING:</b> You must call {@link #stop()} when done, or the motor
     * will keep running indefinitely.
     */
    public void runEject() {
        currentMode = "ejecting";
        applyVolts(-Util.MAX_VOLTS * getSpeedFactor());
    }

    /**
     * Directly stops the intake (for use outside command system).
     */
    public void stop() {
        cleanup();
    }

    //endregion
}

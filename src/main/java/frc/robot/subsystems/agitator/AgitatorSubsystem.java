package frc.robot.subsystems.agitator;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.Agitator.*;

/**
 * Subsystem for the agitator motor.
 * <p>
 * Single NEO motor that feeds balls toward or away from the launcher wheels.
 * Open-loop voltage control. Runs as a separate subsystem so it can be
 * independently scheduled (e.g., start agitator after wheels are at speed).
 */
public class AgitatorSubsystem extends SubsystemBase {

    static final boolean verboseLogging = true;

//region Implementation --------------------------------------------------------

    final AgitatorHardware hardware;
    String currentMode;
    double currentRPM;

    /**
     * Creates an {@link AgitatorSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public AgitatorSubsystem(AgitatorHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        this.currentMode = "idle";

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("RPM", () -> currentRPM, null);
            if (verboseLogging) {
                builder.addDoubleProperty("Amps", hardware::getAmps, null);
            }
        });
    }

    @Override
    public void periodic() {
        currentRPM = hardware.getRPM();
    }

    private double powerToVolts(double powerPercent) {
        return Util.clampVolts((powerPercent / 100.0) * Util.MAX_VOLTS);
    }

    private void cleanup() {
        hardware.stop();
        currentMode = "idle";
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that idles the agitator (default command)
     */
    public Command idleCommand() {
        return run(() -> {
            currentMode = "idle";
            hardware.stop();
        });
    }

    /**
     * Runs agitator forward to feed balls toward the wheels.
     *
     * @return a command that feeds forward
     */
    public Command forwardCommand() {
        return startRun(
            () -> {
                currentMode = "forward";
                hardware.resetConfig();
            },
            () -> hardware.applyVolts(powerToVolts(forwardPower.getAsDouble()))
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Runs agitator forward at feed power (higher than forward, for shooting).
     *
     * @return a command that feeds at shot speed
     */
    public Command feedCommand() {
        return startRun(
            () -> {
                currentMode = "feed";
                hardware.resetConfig();
            },
            () -> hardware.applyVolts(-powerToVolts(feedPower.getAsDouble()))
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Runs agitator at a specified feed power for shooting.
     * Used by distance-based shooting where power varies per shot type.
     *
     * @param powerSupplier supplier for the power percentage (0-100)
     * @return a command that feeds at the specified power
     */
    public Command feedCommandWithPower(DoubleSupplier powerSupplier) {
        return startRun(
            () -> {
                currentMode = "feed";
                hardware.resetConfig();
            },
            () -> hardware.applyVolts(-powerToVolts(powerSupplier.getAsDouble()))
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Runs agitator in reverse to push balls back toward intake side.
     *
     * @return a command that reverses the agitator
     */
    public Command reverseCommand() {
        return startRun(
            () -> {
                currentMode = "reverse";
                hardware.resetConfig();
            },
            () -> hardware.applyVolts(-powerToVolts(forwardPower.getAsDouble()))
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that immediately stops the agitator
     */
    public Command stopCommand() {
        return runOnce(() -> {
            currentMode = "stopped";
            hardware.stop();
        });
    }

    /**
     * Directly stops the motor (for use outside command system).
     */
    public void stop() {
        cleanup();
    }

//endregion
}

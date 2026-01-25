package frc.robot.subsystems.intakefront;

import java.util.Objects;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Config.IntakeFront.*;

/**
 * Subsystem for the front intake mechanism.
 * <p>
 * Uses closed-loop velocity control with separate intake and eject speeds.
 * Includes stall detection to indicate when the hopper is full.
 * <p>
 * Direction is configurable via the {@code IntakeFront/Inverted?} preference.
 */
public class IntakeFrontSubsystem extends SubsystemBase {

    //region Implementation --------------------------------------------------------

    private final IntakeFrontHardware hardware;
    private String currentMode = "idle";
    private double targetSpeedRPS = 0;
    private double currentSpeedRPM = 0;

    // Stall detection
    private final Timer stallTimer = new Timer();
    private boolean isStalled = false;
    private boolean wasStalled = false;
    private Consumer<Boolean> stallCallback = null;

    /**
     * Creates an {@link IntakeFrontSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public IntakeFrontSubsystem(IntakeFrontHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        stallTimer.start();

        // Dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("TargetRPS", () -> targetSpeedRPS, null);
            builder.addDoubleProperty("VelocityRPM", () -> currentSpeedRPM, null);
            builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
            builder.addBooleanProperty("Inverted?", inverted::getAsBoolean, null);
            builder.addBooleanProperty("Stalled?", () -> isStalled, null);
        });
    }

    @Override
    public void periodic() {
        currentSpeedRPM = hardware.getVelocityRPM();
        updateStallDetection();
    }

    /**
     * Sets a callback to be notified when stall state changes.
     * <p>
     * Use this to integrate with LEDs or other alert systems.
     * The callback receives {@code true} when stalled (hopper full),
     * {@code false} when not stalled.
     *
     * @param callback the callback to invoke on stall state change, or null to clear
     */
    public void setStallCallback(Consumer<Boolean> callback) {
        this.stallCallback = callback;
    }

    /**
     * @return true if the intake is currently stalled (likely hopper full)
     */
    public boolean isStalled() {
        return isStalled;
    }

    /**
     * Updates stall detection logic.
     * <p>
     * Stall is detected when:
     * - Motor is being commanded (targetSpeedRPS != 0)
     * - Actual velocity is below threshold
     * - This condition persists for the configured time
     */
    private void updateStallDetection() {
        boolean commanding = Math.abs(targetSpeedRPS) > 0.1;
        double thresholdRPM = stallThresholdRPM.getAsDouble();
        boolean velocityLow = Math.abs(currentSpeedRPM) < thresholdRPM;

        if (commanding && velocityLow) {
            // Might be stalled - check timer
            if (stallTimer.hasElapsed(stallTimeSec.getAsDouble())) {
                isStalled = true;
            }
        } else {
            // Not stalled - reset timer
            stallTimer.reset();
            isStalled = false;
        }

        // Notify callback on state change
        if (isStalled != wasStalled && stallCallback != null) {
            stallCallback.accept(isStalled);
        }
        wasStalled = isStalled;
    }

    /**
     * Applies speed to the intake motor, respecting the inversion setting.
     *
     * @param speedRPS the speed in revolutions per second (positive = intake direction)
     */
    private void applySpeed(double speedRPS) {
        // Apply inversion if configured
        if (inverted.getAsBoolean()) {
            speedRPS = -speedRPS;
        }
        targetSpeedRPS = speedRPS;
        hardware.setSpeed(speedRPS);
    }

    /**
     * Resets the PID controller with current config values.
     */
    private void resetPid() {
        hardware.resetPid(kV.getAsDouble(), kP.getAsDouble());
    }

    /**
     * Cleanup helper - stops motor and resets state.
     */
    private void cleanup() {
        hardware.stop();
        currentMode = "idle";
        targetSpeedRPS = 0;
        stallTimer.reset();
        isStalled = false;
    }

    //endregion

    //region Command factories -----------------------------------------------------

    /**
     * @return a command that stops the intake
     */
    public Command stopCommand() {
        return runOnce(() -> {
            currentMode = "stopped";
            targetSpeedRPS = 0;
            hardware.stop();
        });
    }

    /**
     * @return a command that runs the intake to suck in game pieces
     */
    public Command intakeCommand() {
        return startRun(
            () -> {
                currentMode = "intaking";
                resetPid();
                stallTimer.reset();
            },
            () -> applySpeed(intakeSpeedRPS.getAsDouble())
        ).finallyDo(this::cleanup);
    }

    /**
     * @return a command that runs the intake in reverse (eject)
     */
    public Command ejectCommand() {
        return startRun(
            () -> {
                currentMode = "ejecting";
                resetPid();
                stallTimer.reset();
            },
            () -> applySpeed(-ejectSpeedRPS.getAsDouble())
        ).finallyDo(this::cleanup);
    }

    /**
     * @return a command that idles the intake (stops motor once, then does nothing)
     */
    public Command idleCommand() {
        return startRun(
            () -> {
                // Initialize: stop motor once
                currentMode = "idle";
                targetSpeedRPS = 0;
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
        resetPid();
        applySpeed(intakeSpeedRPS.getAsDouble());
    }

    /**
     * Directly runs the intake in reverse (for use outside command system).
     * <p>
     * <b>WARNING:</b> You must call {@link #stop()} when done, or the motor
     * will keep running indefinitely.
     */
    public void runEject() {
        currentMode = "ejecting";
        resetPid();
        applySpeed(-ejectSpeedRPS.getAsDouble());
    }

    /**
     * Directly stops the intake (for use outside command system).
     */
    public void stop() {
        cleanup();
    }

    //endregion
}

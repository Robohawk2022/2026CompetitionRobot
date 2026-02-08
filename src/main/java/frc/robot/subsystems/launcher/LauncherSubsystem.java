package frc.robot.subsystems.launcher;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.Launcher.*;

/**
 * Subsystem for the launcher mechanism with 3 NEO motors.
 * <p>
 * <b>Motors:</b>
 * <ul>
 *   <li>Agitator - feeds balls from intake to the wheels (open-loop voltage)</li>
 *   <li>Lower wheel - bottom flywheel (onboard velocity PID), doubles as intake</li>
 *   <li>Upper wheel - top flywheel (onboard velocity PID)</li>
 * </ul>
 * <p>
 * <b>Intake:</b> The lower wheel spins backward at a configurable RPM to pull
 * balls in, while the agitator feeds them forward. No separate intake motor.
 * <p>
 * <b>Arc control:</b> By running the upper and lower wheels at different
 * speeds, we create spin on the ball (Magnus effect). Upper faster = backspin
 * = higher arc. Lower faster = topspin = flatter shot.
 * <p>
 * <b>Velocity control:</b> The wheels use SparkMax onboard PID (1kHz) with
 * feedforward (kV) and proportional/derivative feedback (kP/kD). Tune kV first
 * to get close, then add kP to correct the remaining error. Gains are
 * live-tunable via Preferences and applied when commands start.
 */
public class LauncherSubsystem extends SubsystemBase {

    static final boolean verboseLogging = true;

    /**
     * Shot presets with different wheel RPM targets for arc control.
     * <p>
     * Each preset defines an RPM for lower and upper wheels.
     * The ratio between them controls the ball's arc.
     */
    public enum ShotPreset {
        /** High arc - upper wheel faster for backspin */
        HIGH_ARC,
        /** Flat shot - lower wheel faster for topspin */
        FLAT,
        /** Equal speed - neutral spin */
        NEUTRAL;

        public double lowerRPM() {
            return switch (this) {
                case HIGH_ARC -> highArcLowerRPM.getAsDouble();
                case FLAT -> flatLowerRPM.getAsDouble();
                case NEUTRAL -> neutralRPM.getAsDouble();
            };
        }

        public double upperRPM() {
            return switch (this) {
                case HIGH_ARC -> highArcUpperRPM.getAsDouble();
                case FLAT -> flatUpperRPM.getAsDouble();
                case NEUTRAL -> neutralRPM.getAsDouble();
            };
        }
    }

//region Implementation --------------------------------------------------------

    final LauncherHardware hardware;

    String currentMode;
    double agitatorRPM, lowerWheelRPM, upperWheelRPM;
    double lowerTargetRPM, upperTargetRPM;

    /**
     * Creates a {@link LauncherSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public LauncherSubsystem(LauncherHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        this.currentMode = "idle";

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);

            // wheel RPMs grouped alphabetically by category
            builder.addDoubleProperty("LowerWheelCurrent", () -> lowerWheelRPM, null);
            builder.addDoubleProperty("LowerWheelTarget", () -> lowerTargetRPM, null);
            builder.addDoubleProperty("UpperWheelCurrent", () -> upperWheelRPM, null);
            builder.addDoubleProperty("UpperWheelTarget", () -> upperTargetRPM, null);

            builder.addBooleanProperty("AtSpeed?", this::atSpeed, null);

            builder.addDoubleProperty("AgitatorRPM", () -> agitatorRPM, null);

            if (verboseLogging) {
                builder.addDoubleProperty("AgitatorAmps", hardware::getAgitatorAmps, null);
                builder.addDoubleProperty("LowerWheelAmps", hardware::getLowerWheelAmps, null);
                builder.addDoubleProperty("UpperWheelAmps", hardware::getUpperWheelAmps, null);
            }
        });
    }

    @Override
    public void periodic() {
        agitatorRPM = hardware.getAgitatorRPM();
        lowerWheelRPM = hardware.getLowerWheelRPM();
        upperWheelRPM = hardware.getUpperWheelRPM();
    }

    /**
     * @return true if both wheels are within tolerance of their target RPM
     */
    public boolean atSpeed() {
        double tol = tolerance.getAsDouble();
        return lowerTargetRPM > 0
            && Math.abs(lowerWheelRPM - lowerTargetRPM) <= tol
            && Math.abs(upperWheelRPM - upperTargetRPM) <= tol;
    }

    /**
     * Sends RPM targets to both wheels via onboard PID.
     */
    private void driveWheels(double lowerTarget, double upperTarget) {
        lowerTargetRPM = lowerTarget;
        upperTargetRPM = upperTarget;
        hardware.setLowerWheelRPM(lowerTarget);
        hardware.setUpperWheelRPM(upperTarget);
    }

    /**
     * Pushes current PID gains from Config to the SparkMax onboard controllers.
     * Called at command start to pick up any live-tuned values.
     */
    private void applyPIDGains() {
        hardware.resetPID(kV.getAsDouble(), kP.getAsDouble(), kD.getAsDouble());
    }

    private double powerToVolts(double powerPercent) {
        return Util.clampVolts((powerPercent / 100.0) * Util.MAX_VOLTS);
    }

    private void cleanup() {
        hardware.stopAll();
        lowerTargetRPM = 0;
        upperTargetRPM = 0;
        currentMode = "idle";
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that idles all motors (default command)
     */
    public Command idleCommand() {
        return run(() -> {
            currentMode = "idle";
            hardware.stopAll();
        });
    }

    /**
     * Intake: lower wheel spins backward (negative RPM) to pull balls in,
     * agitator feeds forward, upper wheel off.
     *
     * @return a command that runs the intake
     */
    public Command intakeCommand() {
        return startRun(
            () -> {
                currentMode = "intake";
                applyPIDGains();
            },
            () -> {
                // lower wheel backward at intake speed (negative = reverse)
                driveWheels(-intakeSpeedRPM.getAsDouble(), 0);
                hardware.applyAgitatorVolts(powerToVolts(agitatorPower.getAsDouble()));
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Eject: lower wheel spins backward, agitator reversed.
     *
     * @return a command that reverses to eject a ball
     */
    public Command ejectCommand() {
        return startRun(
            () -> {
                currentMode = "eject";
                applyPIDGains();
            },
            () -> {
                driveWheels(-intakeSpeedRPM.getAsDouble(), 0);
                hardware.applyAgitatorVolts(-powerToVolts(agitatorPower.getAsDouble()));
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins up the wheels without feeding a ball. Use this to pre-spin
     * before shooting. Uses onboard velocity PID.
     *
     * @param preset the shot preset controlling wheel RPM targets
     * @return a command that spins the wheels to the preset RPMs
     */
    public Command spinUpCommand(ShotPreset preset) {
        return startRun(
            () -> {
                currentMode = "spin-up:" + preset.name();
                applyPIDGains();
            },
            () -> {
                hardware.applyAgitatorVolts(0);
                driveWheels(preset.lowerRPM(), preset.upperRPM());
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins up the wheels and feeds the ball using the agitator.
     * Uses onboard velocity PID on the wheels.
     *
     * @param preset the shot preset controlling wheel RPM targets
     * @return a command that shoots at the preset arc
     */
    public Command shootCommand(ShotPreset preset) {
        return startRun(
            () -> {
                currentMode = "shoot:" + preset.name();
                applyPIDGains();
            },
            () -> {
                hardware.applyAgitatorVolts(powerToVolts(feedPower.getAsDouble()));
                driveWheels(preset.lowerRPM(), preset.upperRPM());
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that immediately stops all motors
     */
    public Command stopCommand() {
        return runOnce(() -> {
            currentMode = "stopped";
            hardware.stopAll();
        });
    }

    /**
     * Directly stops all motors (for use outside command system, e.g. disabledInit).
     */
    public void stop() {
        cleanup();
    }

//endregion
}

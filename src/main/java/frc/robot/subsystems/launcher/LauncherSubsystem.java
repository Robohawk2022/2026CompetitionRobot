package frc.robot.subsystems.launcher;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Config.Launcher.*;

/**
 * Subsystem for the launcher wheels (lower + upper).
 * <p>
 * <b>Motors:</b>
 * <ul>
 *   <li>Lower wheel - bottom flywheel (onboard velocity PID), doubles as intake</li>
 *   <li>Upper wheel - top flywheel (onboard velocity PID)</li>
 * </ul>
 * <p>
 * <b>Intake:</b> The lower wheel spins backward at a configurable RPM to pull
 * balls in. No separate intake motor.
 * <p>
 * <b>Arc control:</b> By running the upper and lower wheels at different
 * speeds, we create spin on the ball (Magnus effect). Upper faster = backspin
 * = higher arc. Lower faster = topspin = flatter shot.
 * <p>
 * <b>Velocity control:</b> Each wheel has its own SparkMax onboard PID (1kHz)
 * with feedforward (kV) and proportional feedback (kP). Gains are live-tunable
 * via Preferences and applied when commands start.
 * <p>
 * <b>Note:</b> The agitator is a separate subsystem ({@code AgitatorSubsystem}).
 * Shoot sequences are composed by combining both subsystems.
 */
public class LauncherSubsystem extends SubsystemBase {

    static final boolean verboseLogging = true;

    /**
     * Shot presets with different wheel RPM targets for arc control.
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
    double lowerWheelRPM, upperWheelRPM;
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

            builder.addDoubleProperty("LowerWheelCurrent", () -> lowerWheelRPM, null);
            builder.addDoubleProperty("LowerWheelTarget", () -> lowerTargetRPM, null);
            builder.addDoubleProperty("LowerWheelError", () -> lowerTargetRPM - lowerWheelRPM, null);
            builder.addDoubleProperty("UpperWheelCurrent", () -> upperWheelRPM, null);
            builder.addDoubleProperty("UpperWheelTarget", () -> upperTargetRPM, null);
            builder.addDoubleProperty("UpperWheelError", () -> upperTargetRPM - upperWheelRPM, null);

            builder.addBooleanProperty("AtSpeed?", this::atSpeed, null);

            if (verboseLogging) {
                builder.addDoubleProperty("LowerWheelAmps", hardware::getLowerWheelAmps, null);
                builder.addDoubleProperty("UpperWheelAmps", hardware::getUpperWheelAmps, null);
            }
        });
    }

    @Override
    public void periodic() {
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

    /** Maximum safe RPM for NEO motors (free speed is 5676) */
    static final double MAX_RPM = 5700;

    /**
     * Sends RPM targets to both wheels via onboard PID.
     * Clamps values to NEO safe range to prevent runaway from bad config.
     */
    private void driveWheels(double lowerTarget, double upperTarget) {
        lowerTargetRPM = MathUtil.clamp(lowerTarget, -MAX_RPM, MAX_RPM);
        upperTargetRPM = MathUtil.clamp(upperTarget, -MAX_RPM, MAX_RPM);
        hardware.setLowerWheelRPM(lowerTargetRPM);
        hardware.setUpperWheelRPM(upperTargetRPM);
    }

    /**
     * Pushes current PID gains from Config to the SparkMax onboard controllers.
     */
    private void applyPIDGains() {
        hardware.resetLowerPID(lowerKV.getAsDouble(), lowerKP.getAsDouble());
        hardware.resetUpperPID(upperKV.getAsDouble(), upperKP.getAsDouble());
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
     * @return a command that idles both wheels (default command)
     */
    public Command idleCommand() {
        return run(() -> {
            currentMode = "idle";
            hardware.stopAll();
        });
    }

    /**
     * Spins the lower wheel backward for intake. Upper wheel off.
     *
     * @return a command that runs the lower wheel for intake
     */
    public Command intakeWheelCommand() {
        return startRun(
            () -> {
                currentMode = "intake";
                applyPIDGains();
            },
            () -> driveWheels(intakeSpeedRPM.getAsDouble(), -intakeSpeedRPM.getAsDouble())
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins the lower wheel backward to push ball back out. Upper wheel off.
     *
     * @return a command that runs the lower wheel for eject
     */
    public Command ejectWheelCommand() {
        return startRun(
            () -> {
                currentMode = "eject";
                applyPIDGains();
            },
            () -> driveWheels(-intakeSpeedRPM.getAsDouble(), 0)
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins both wheels at the preset RPMs. Runs continuously until
     * interrupted — does NOT stop on its own.
     *
     * @param preset the shot preset controlling wheel RPM targets
     * @return a command that spins the wheels
     */
    public Command spinUpCommand(ShotPreset preset) {
        return startRun(
            () -> {
                currentMode = "spin-up:" + preset.name();
                applyPIDGains();
            },
            () -> driveWheels(preset.lowerRPM(), preset.upperRPM())
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that immediately stops both wheels
     */
    public Command stopCommand() {
        return runOnce(() -> cleanup());
    }

    /**
     * Directly stops both wheels (for use outside command system).
     */
    public void stop() {
        cleanup();
    }

//endregion
}

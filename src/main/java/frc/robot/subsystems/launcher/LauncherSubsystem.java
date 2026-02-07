package frc.robot.subsystems.launcher;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PDController;
import frc.robot.util.Util;

import static frc.robot.Config.Launcher.*;

/**
 * Subsystem for the launcher mechanism with 4 NEO motors.
 * <p>
 * <b>Motors:</b>
 * <ul>
 *   <li>Intake - pulls balls into the launcher (open-loop voltage)</li>
 *   <li>Agitator - feeds balls from intake to the wheels (open-loop voltage)</li>
 *   <li>Lower wheel - bottom flywheel (closed-loop velocity)</li>
 *   <li>Upper wheel - top flywheel (closed-loop velocity)</li>
 * </ul>
 * <p>
 * <b>Arc control:</b> By running the upper and lower wheels at different
 * speeds, we create spin on the ball (Magnus effect). Upper faster = backspin
 * = higher arc. Lower faster = topspin = flatter shot.
 * <p>
 * <b>Velocity control:</b> The wheels use feedforward (kV * targetRPM) plus
 * PD feedback to hold a target RPM. Tune kV first to get close, then add kP
 * to correct the remaining error.
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
    final PDController lowerPID;
    final PDController upperPID;

    String currentMode;
    double intakeRPM, agitatorRPM, lowerWheelRPM, upperWheelRPM;
    double lowerTargetRPM, upperTargetRPM;
    double lowerVolts, upperVolts;
    double lowerFeedforward, upperFeedforward;
    double lowerFeedback, upperFeedback;

    /**
     * Creates a {@link LauncherSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public LauncherSubsystem(LauncherHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        this.currentMode = "idle";
        this.lowerPID = new PDController(kP, kD, tolerance);
        this.upperPID = new PDController(kP, kD, tolerance);

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);

            // wheel RPMs grouped alphabetically by category
            builder.addDoubleProperty("LowerWheelCurrent", () -> lowerWheelRPM, null);
            builder.addDoubleProperty("LowerWheelTarget", () -> lowerTargetRPM, null);
            builder.addDoubleProperty("UpperWheelCurrent", () -> upperWheelRPM, null);
            builder.addDoubleProperty("UpperWheelTarget", () -> upperTargetRPM, null);

            builder.addBooleanProperty("AtSpeed?", this::atSpeed, null);

            builder.addDoubleProperty("IntakeRPM", () -> intakeRPM, null);
            builder.addDoubleProperty("AgitatorRPM", () -> agitatorRPM, null);

            if (verboseLogging) {
                builder.addDoubleProperty("LowerWheelFeedforward", () -> lowerFeedforward, null);
                builder.addDoubleProperty("LowerWheelFeedback", () -> lowerFeedback, null);
                builder.addDoubleProperty("LowerWheelVolts", () -> lowerVolts, null);
                builder.addDoubleProperty("UpperWheelFeedforward", () -> upperFeedforward, null);
                builder.addDoubleProperty("UpperWheelFeedback", () -> upperFeedback, null);
                builder.addDoubleProperty("UpperWheelVolts", () -> upperVolts, null);
                builder.addDoubleProperty("IntakeAmps", hardware::getIntakeAmps, null);
                builder.addDoubleProperty("AgitatorAmps", hardware::getAgitatorAmps, null);
                builder.addDoubleProperty("LowerWheelAmps", hardware::getLowerWheelAmps, null);
                builder.addDoubleProperty("UpperWheelAmps", hardware::getUpperWheelAmps, null);
            }
        });
    }

    @Override
    public void periodic() {
        intakeRPM = hardware.getIntakeRPM();
        agitatorRPM = hardware.getAgitatorRPM();
        lowerWheelRPM = hardware.getLowerWheelRPM();
        upperWheelRPM = hardware.getUpperWheelRPM();
    }

    /**
     * @return true if both wheels are within tolerance of their target RPM
     */
    public boolean atSpeed() {
        return lowerTargetRPM > 0
            && lowerPID.withinTolerance(lowerWheelRPM, lowerTargetRPM)
            && upperPID.withinTolerance(upperWheelRPM, upperTargetRPM);
    }

    /**
     * Applies closed-loop velocity control to a wheel.
     * Voltage = feedforward (kV * target) + feedback (PD on error).
     */
    private double calculateWheelVolts(PDController pid, double currentRPM, double targetRPM) {
        double feedforward = kV.getAsDouble() * targetRPM;
        double feedback = pid.calculate(currentRPM, targetRPM);
        return Util.clampVolts(feedforward + feedback);
    }

    /**
     * Runs closed-loop control on both wheels and stores telemetry.
     */
    private void driveWheels(double lowerTarget, double upperTarget) {
        lowerTargetRPM = lowerTarget;
        upperTargetRPM = upperTarget;

        lowerFeedforward = kV.getAsDouble() * lowerTarget;
        lowerFeedback = lowerPID.calculate(lowerWheelRPM, lowerTarget);
        lowerVolts = Util.clampVolts(lowerFeedforward + lowerFeedback);

        upperFeedforward = kV.getAsDouble() * upperTarget;
        upperFeedback = upperPID.calculate(upperWheelRPM, upperTarget);
        upperVolts = Util.clampVolts(upperFeedforward + upperFeedback);

        hardware.applyLowerWheelVolts(lowerVolts);
        hardware.applyUpperWheelVolts(upperVolts);
    }

    private double powerToVolts(double powerPercent) {
        return Util.clampVolts((powerPercent / 100.0) * Util.MAX_VOLTS);
    }

    private void cleanup() {
        hardware.stopAll();
        lowerTargetRPM = 0;
        upperTargetRPM = 0;
        lowerVolts = 0;
        upperVolts = 0;
        lowerFeedforward = 0;
        upperFeedforward = 0;
        lowerFeedback = 0;
        upperFeedback = 0;
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
     * @return a command that runs the intake motor to pull balls in
     */
    public Command intakeCommand() {
        return startRun(
            () -> currentMode = "intake",
            () -> {
                hardware.applyIntakeVolts(powerToVolts(intakePower.getAsDouble()));
                hardware.applyAgitatorVolts(powerToVolts(agitatorPower.getAsDouble()));
                hardware.applyLowerWheelVolts(0);
                hardware.applyUpperWheelVolts(0);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that reverses intake and agitator to eject a ball
     */
    public Command ejectCommand() {
        return startRun(
            () -> currentMode = "eject",
            () -> {
                hardware.applyIntakeVolts(-powerToVolts(intakePower.getAsDouble()));
                hardware.applyAgitatorVolts(-powerToVolts(agitatorPower.getAsDouble()));
                hardware.applyLowerWheelVolts(0);
                hardware.applyUpperWheelVolts(0);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins up the wheels without feeding a ball. Use this to pre-spin
     * before shooting. Uses closed-loop velocity control.
     *
     * @param preset the shot preset controlling wheel RPM targets
     * @return a command that spins the wheels to the preset RPMs
     */
    public Command spinUpCommand(ShotPreset preset) {
        return startRun(
            () -> {
                currentMode = "spin-up:" + preset.name();
                lowerPID.reset();
                upperPID.reset();
            },
            () -> {
                hardware.applyIntakeVolts(0);
                hardware.applyAgitatorVolts(0);
                driveWheels(preset.lowerRPM(), preset.upperRPM());
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins up the wheels and feeds the ball using the agitator.
     * Uses closed-loop velocity control on the wheels.
     *
     * @param preset the shot preset controlling wheel RPM targets
     * @return a command that shoots at the preset arc
     */
    public Command shootCommand(ShotPreset preset) {
        return startRun(
            () -> {
                currentMode = "shoot:" + preset.name();
                lowerPID.reset();
                upperPID.reset();
            },
            () -> {
                hardware.applyIntakeVolts(powerToVolts(intakePower.getAsDouble()));
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

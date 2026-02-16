package frc.robot.subsystems.launcher;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import edu.wpi.first.math.MathUtil;

import static frc.robot.Config.Launcher.*;

/**
 * Subsystem for the launcher: 2 feeder motors + 1 shooter motor.
 * <p>
 * <b>Motors:</b>
 * <ul>
 *   <li>Feeder Left - bottom left (onboard velocity PID), spins inward</li>
 *   <li>Feeder Right - bottom right (onboard velocity PID), spins inward</li>
 *   <li>Shooter - top flywheel (onboard velocity PID), flings balls out</li>
 * </ul>
 * <p>
 * <b>Intake:</b> Both feeders spin inward at intake RPM to pull balls in.
 * <p>
 * <b>Shooting:</b> Feeders spin inward at feed RPM while shooter spins at
 * shoot RPM. All three must reach target before feeding begins.
 */
public class LauncherSubsystem extends SubsystemBase {

    static final boolean verboseLogging = true;

//region Implementation --------------------------------------------------------

    final LauncherHardware hardware;

    String currentMode;
    double feederLeftRPM, feederRightRPM, currentShooterRPM;
    double feederLeftTarget, feederRightTarget, shooterTarget;

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
            builder.addBooleanProperty("AtSpeed?", this::atSpeed, null);

            builder.addDoubleProperty("FeederLeftCurrent", () -> Util.chopDigits(feederLeftRPM), null);
            builder.addDoubleProperty("FeederLeftTarget", () -> feederLeftTarget, null);
            builder.addDoubleProperty("FeederRightCurrent", () -> Util.chopDigits(feederRightRPM), null);
            builder.addDoubleProperty("FeederRightTarget", () -> feederRightTarget, null);
            builder.addDoubleProperty("ShooterCurrent", () -> Util.chopDigits(currentShooterRPM), null);
            builder.addDoubleProperty("ShooterTarget", () -> shooterTarget, null);

            if (verboseLogging) {
                builder.addDoubleProperty("FeederLeftAmps", hardware::getFeederLeftAmps, null);
                builder.addDoubleProperty("FeederRightAmps", hardware::getFeederRightAmps, null);
                builder.addDoubleProperty("ShooterAmps", hardware::getShooterAmps, null);
            }
        });
    }

    @Override
    public void periodic() {
        feederLeftRPM = hardware.getFeederLeftRPM();
        feederRightRPM = hardware.getFeederRightRPM();
        currentShooterRPM = hardware.getShooterRPM();
    }

    /** @return the current command mode (e.g. "idle", "intake", "shoot") */
    public String getCurrentMode() {
        return currentMode;
    }

    /**
     * @return true if all active motors are within tolerance of their target RPM
     */
    public boolean atSpeed() {
        double tol = tolerance.getAsDouble();
        // only check motors that have a non-zero target
        boolean feedersOk = feederLeftTarget == 0
            || (Math.abs(feederLeftRPM - feederLeftTarget) <= tol
                && Math.abs(feederRightRPM - feederRightTarget) <= tol);
        boolean shooterOk = shooterTarget == 0
            || Math.abs(currentShooterRPM - shooterTarget) <= tol;
        // at least one motor must be running
        return (feederLeftTarget != 0 || shooterTarget != 0) && feedersOk && shooterOk;
    }

    /** Maximum safe RPM for NEO motors (free speed is 5676) */
    static final double MAX_RPM = 5700;

    /**
     * Sends RPM targets to all three motors via onboard PID.
     * Clamps values to NEO safe range.
     */
    private void driveMotors(double feederLeft, double feederRight, double shooter) {
        feederLeftTarget = MathUtil.clamp(feederLeft, -MAX_RPM, MAX_RPM);
        feederRightTarget = MathUtil.clamp(feederRight, -MAX_RPM, MAX_RPM);
        shooterTarget = MathUtil.clamp(shooter, -MAX_RPM, MAX_RPM);
        hardware.setFeederLeftRPM(feederLeftTarget);
        hardware.setFeederRightRPM(feederRightTarget);
        hardware.setShooterRPM(shooterTarget);
    }

    /**
     * Pushes current PID gains from Config to the SparkMax onboard controllers.
     */
    private void applyPIDGains() {
        hardware.resetFeederLeftPID(
            feederLeftKV.getAsDouble(), feederLeftKP.getAsDouble(), 0, feederLeftKD.getAsDouble());
        hardware.resetFeederRightPID(
            feederRightKV.getAsDouble(), feederRightKP.getAsDouble(), 0, feederRightKD.getAsDouble());
        hardware.resetShooterPID(
            shooterKV.getAsDouble(), shooterKP.getAsDouble(), 0, shooterKD.getAsDouble());
    }

    private void cleanup() {
        hardware.stopAll();
        feederLeftTarget = 0;
        feederRightTarget = 0;
        shooterTarget = 0;
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
     * Spins both feeders inward at intake RPM. Shooter off.
     *
     * @return a command that runs the feeders for intake
     */
    public Command intakeCommand() {
        return startRun(
            () -> {
                currentMode = "intake";
                applyPIDGains();
            },
            () -> driveMotors(FEEDER_RPM, FEEDER_RPM, SHOOTER_INTAKE_RPM)
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins both feeders outward (reverse) to eject balls. Shooter off.
     *
     * @return a command that runs the feeders in reverse for eject
     */
    public Command ejectCommand() {
        return startRun(
            () -> {
                currentMode = "eject";
                applyPIDGains();
            },
            () -> driveMotors(-FEEDER_RPM, -FEEDER_RPM, 0)
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * Spins feeders inward at feed RPM and shooter at shoot RPM.
     * Runs continuously until interrupted.
     *
     * @return a command that runs all motors for shooting
     */
    public Command shootCommand() {
        return startRun(
            () -> {
                currentMode = "shoot";
                applyPIDGains();
            },
            () -> driveMotors(FEED_SHOOT_RPM, FEED_SHOOT_RPM, shooterRPM.getAsDouble())
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that immediately stops all motors
     */
    public Command stopCommand() {
        return runOnce(this::cleanup);
    }

    /**
     * Directly stops all motors (for use outside command system).
     */
    public void stop() {
        cleanup();
    }

//endregion
}

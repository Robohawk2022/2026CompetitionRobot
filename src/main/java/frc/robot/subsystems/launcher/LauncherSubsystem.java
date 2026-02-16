package frc.robot.subsystems.launcher;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Field;
import frc.robot.util.Util;

import static frc.robot.Config.Launcher.*;

/**
 * Subsystem for the launcher: 2 feeder motors + 1 shooter motor + 1 flapper.
 * <p>
 * <b>Motors:</b>
 * <ul>
 *   <li>Feeder Left - bottom left (onboard velocity PID), spins inward</li>
 *   <li>Feeder Right - bottom right (onboard velocity PID), spins inward</li>
 *   <li>Shooter - top flywheel (onboard velocity PID), flings balls out</li>
 *   <li>Flapper - keeps balls from re-entering during intake</li>
 * </ul>
 * <p>
 * Supports distance-based shooter RPM via an interpolation table when a
 * pose supplier is provided.
 */
public class LauncherSubsystem extends SubsystemBase {

    static final boolean verboseLogging = true;

//region Implementation --------------------------------------------------------

    final LauncherHardware hardware;
    final Supplier<Pose2d> poseSupplier;
    final InterpolatingDoubleTreeMap distanceToRPM;

    String currentMode;
    double feederLeftRPM, feederRightRPM, currentShooterRPM, flapperRPM;
    double feederLeftTarget, feederRightTarget, shooterTarget, flapperTarget;
    double distanceToHopper;

    /**
     * Creates a {@link LauncherSubsystem}.
     *
     * @param hardware the hardware interface (required)
     * @param poseSupplier supplier for robot pose (null if no pose available)
     */
    public LauncherSubsystem(LauncherHardware hardware, Supplier<Pose2d> poseSupplier) {
        this.hardware = Objects.requireNonNull(hardware);
        this.poseSupplier = poseSupplier;
        this.currentMode = "idle";
        this.distanceToHopper = -1;

        // distance (feet) -> shooter RPM interpolation table
        distanceToRPM = new InterpolatingDoubleTreeMap();
        distanceToRPM.put(CLOSE_DISTANCE_FEET, CLOSE_RPM);

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addBooleanProperty("AtSpeed?", this::atSpeed, null);

            builder.addDoubleProperty("DistanceToHopper", () -> Util.chopDigits(distanceToHopper), null);

            builder.addDoubleProperty("FeederLeftCurrent", () -> Util.chopDigits(feederLeftRPM), null);
            builder.addDoubleProperty("FeederLeftTarget", () -> feederLeftTarget, null);
            builder.addDoubleProperty("FeederRightCurrent", () -> Util.chopDigits(feederRightRPM), null);
            builder.addDoubleProperty("FeederRightTarget", () -> feederRightTarget, null);
            builder.addDoubleProperty("ShooterCurrent", () -> Util.chopDigits(currentShooterRPM), null);
            builder.addDoubleProperty("ShooterTarget", () -> shooterTarget, null);
            builder.addDoubleProperty("FlapperCurrent", () -> Util.chopDigits(flapperRPM), null);
            builder.addDoubleProperty("FlapperTarget", () -> flapperTarget, null);

            if (verboseLogging) {
                builder.addDoubleProperty("FeederLeftAmps", hardware::getFeederLeftAmps, null);
                builder.addDoubleProperty("FeederRightAmps", hardware::getFeederRightAmps, null);
                builder.addDoubleProperty("ShooterAmps", hardware::getShooterAmps, null);
                builder.addDoubleProperty("FlapperAmps", hardware::getFlapperAmps, null);
            }
        });
    }

    /**
     * Creates a {@link LauncherSubsystem} without pose awareness.
     *
     * @param hardware the hardware interface (required)
     */
    public LauncherSubsystem(LauncherHardware hardware) {
        this(hardware, null);
    }

    @Override
    public void periodic() {
        feederLeftRPM = hardware.getFeederLeftRPM();
        feederRightRPM = hardware.getFeederRightRPM();
        currentShooterRPM = hardware.getShooterRPM();
        flapperRPM = hardware.getFlapperRPM();

        if (poseSupplier != null) {
            Pose2d pose = poseSupplier.get();
            distanceToHopper = Util.feetBetween(pose, Field.getHubCenter());
        }
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
        boolean flapperOk = flapperTarget == 0
            || Math.abs(flapperRPM - flapperTarget) <= tol;
        // at least one motor must be running
        return (feederLeftTarget != 0 || shooterTarget != 0 || flapperTarget != 0)
            && feedersOk && shooterOk && flapperOk;
    }

    /** Maximum safe RPM for NEO motors (free speed is 5676) */
    static final double MAX_RPM = 5700;

    /**
     * Sends RPM targets to all four motors via onboard PID.
     * Clamps values to NEO safe range.
     */
    private void driveMotors(double feederLeft, double feederRight, double shooter, double flapper) {
        feederLeftTarget = MathUtil.clamp(feederLeft, -MAX_RPM, MAX_RPM);
        feederRightTarget = MathUtil.clamp(feederRight, -MAX_RPM, MAX_RPM);
        shooterTarget = MathUtil.clamp(shooter, -MAX_RPM, MAX_RPM);
        flapperTarget = MathUtil.clamp(flapper, -MAX_RPM, MAX_RPM);
        hardware.setFeederLeftRPM(feederLeftTarget);
        hardware.setFeederRightRPM(feederRightTarget);
        hardware.setShooterRPM(shooterTarget);
        hardware.setFlapperRPM(flapperTarget);
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
        hardware.resetFlapperPID(
            flapperKV.getAsDouble(), flapperKP.getAsDouble(), 0, flapperKD.getAsDouble());
    }

    private void cleanup() {
        hardware.stopAll();
        feederLeftTarget = 0;
        feederRightTarget = 0;
        shooterTarget = 0;
        flapperTarget = 0;
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
            () -> driveMotors(FEEDER_RPM, FEEDER_RPM, SHOOTER_INTAKE_RPM, FLAPPER_RPM)
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
            () -> driveMotors(-FEEDER_RPM, -FEEDER_RPM, 0, 0)
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
            () -> driveMotors(FEED_SHOOT_RPM, FEED_SHOOT_RPM, shooterRPM.getAsDouble(), 0)
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return the current distance to the hopper in feet, or -1 if no pose available
     */
    public double getDistanceToHopper() {
        return distanceToHopper;
    }

    /**
     * @param distanceFeet distance to hopper in feet
     * @return the interpolated shooter RPM for the given distance
     */
    public double getTargetRPMForDistance(double distanceFeet) {
        return distanceToRPM.get(distanceFeet);
    }

    /**
     * Like {@link #shootCommand()} but uses distance-based RPM from the
     * interpolation table instead of the fixed Config value.
     * Falls back to Config shooterRPM if no pose supplier is available.
     *
     * @return a command that shoots with distance-based RPM
     */
    public Command autoShootCommand() {
        return startRun(
            () -> {
                currentMode = "autoShoot";
                applyPIDGains();
            },
            () -> {
                double rpm = (poseSupplier != null && distanceToHopper >= 0)
                    ? distanceToRPM.get(distanceToHopper)
                    : shooterRPM.getAsDouble();
                driveMotors(FEED_SHOOT_RPM, FEED_SHOOT_RPM, rpm, 0);
            }
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

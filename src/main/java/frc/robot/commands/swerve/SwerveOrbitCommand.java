package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Util;

import static frc.robot.Config.Swerve.*;
import static frc.robot.Config.Orbit.*;

/**
 * Orbit drive command that orbits around the target (Tower) while facing it.
 * <p>
 * Maps controls as follows:
 * <ul>
 *   <li>Left stick Y: Radial movement (toward/away from target)</li>
 *   <li>Left stick X: Tangential movement (orbit around target)</li>
 *   <li>Right stick X: Fine heading adjustment</li>
 *   <li>Triggers: Turbo/sniper speed modifiers</li>
 * </ul>
 */
public class SwerveOrbitCommand extends Command {

    private final SwerveSubsystem swerve;
    private final GameController controller;

    // heading control state
    private double previousHeadingError;
    private boolean initialLockAcquired;

    /**
     * Creates a {@link SwerveOrbitCommand}.
     *
     * @param swerve     the swerve subsystem (required)
     * @param controller the game controller for driver input (required)
     */
    public SwerveOrbitCommand(SwerveSubsystem swerve, GameController controller) {
        this.swerve = Objects.requireNonNull(swerve);
        this.controller = Objects.requireNonNull(controller);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        previousHeadingError = 0;
        initialLockAcquired = false;
    }

    /**
     * @return the target point (Tower position for 2026) for the current alliance
     */
    public Translation2d getTargetPoint() {
        if (Util.isRedAlliance()) {
            return new Translation2d(redTargetX.getAsDouble(), redTargetY.getAsDouble());
        } else {
            return new Translation2d(blueTargetX.getAsDouble(), blueTargetY.getAsDouble());
        }
    }

    @Override
    public void execute() {
        // read speed limits from config
        double maxSpeedMps = Units.feetToMeters(maxSpeedFps.getAsDouble());
        double maxRotationRps = Math.toRadians(maxRotationDps.getAsDouble());

        // get orbit point for current alliance
        Translation2d orbitPoint = getTargetPoint();
        Pose2d robotPose = swerve.getPose();
        Translation2d robotPos = robotPose.getTranslation();

        // vector from robot to orbit point
        Translation2d toOrbit = orbitPoint.minus(robotPos);
        double distance = toOrbit.getNorm();

        // prevent singularity when too close to orbit point
        double minDist = minDistance.getAsDouble();
        if (distance < minDist) {
            swerve.driveRobotRelative("orbit", Util.ZERO_SPEED);
            return;
        }

        // unit vectors: radial (toward orbit point) and tangent (perpendicular, CCW)
        Translation2d radialUnit = toOrbit.div(distance);
        Translation2d tangentUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX());

        // get controller inputs with deadband and inversion
        double radialInput = -MathUtil.applyDeadband(controller.getLeftY(), deadzone.getAsDouble());
        double tangentInput = -MathUtil.applyDeadband(controller.getLeftX(), deadzone.getAsDouble());
        double rotInput = -MathUtil.applyDeadband(controller.getRightX(), deadzone.getAsDouble());

        // clamp inputs
        radialInput = MathUtil.clamp(radialInput, -1.0, 1.0);
        tangentInput = MathUtil.clamp(tangentInput, -1.0, 1.0);
        rotInput = MathUtil.clamp(rotInput, -1.0, 1.0);

        // convert to velocities (in meters per second)
        double radialSpeed = radialInput * Units.feetToMeters(maxRadialSpeedFps.getAsDouble());
        double tangentSpeed = tangentInput * Units.feetToMeters(maxTangentSpeedFps.getAsDouble());

        // check trigger thresholds
        boolean sniperActive = controller.leftTriggerSupplier().getAsBoolean();
        boolean turboActive = controller.rightTriggerSupplier().getAsBoolean();

        // apply speed modifiers (sniper takes priority)
        if (sniperActive) {
            double sf = sniperFactor.getAsDouble();
            radialSpeed *= sf;
            tangentSpeed *= sf;
        } else if (turboActive) {
            double tf = turboFactor.getAsDouble();
            radialSpeed *= tf;
            tangentSpeed *= tf;
        }

        // heading control: always face the orbit point
        double targetHeadingDeg = Math.toDegrees(Math.atan2(toOrbit.getY(), toOrbit.getX()));
        double currentHeadingDeg = robotPose.getRotation().getDegrees();
        double headingError = Util.degreeModulus(targetHeadingDeg - currentHeadingDeg);

        // check if we're locked on target (only matters on initial activation)
        // once locked, stays unlocked for the rest of the command
        if (!initialLockAcquired && Math.abs(headingError) < lockThreshold.getAsDouble()) {
            initialLockAcquired = true;
        }
        boolean canMove = initialLockAcquired;

        // combine radial and tangent into field-relative velocity
        // suppress translation until locked on target
        double vxField = 0;
        double vyField = 0;
        if (canMove) {
            vxField = radialUnit.getX() * radialSpeed + tangentUnit.getX() * tangentSpeed;
            vyField = radialUnit.getY() * radialSpeed + tangentUnit.getY() * tangentSpeed;
        }

        // feedforward: predict rotation rate needed for tangential movement
        // only apply when moving (canMove), otherwise just use PD to lock on target
        double feedforwardDps = canMove ? Math.toDegrees(tangentSpeed / distance) : 0;

        // calculate rotation with PD control and saturation to max
        double maxOmegaDps = Math.toDegrees(maxRotationRps);
        double omegaDps = feedforwardDps + headingKP.getAsDouble() * headingError;

        // add D-term: rate of error change (degrees per second)
        double errorRate = (headingError - previousHeadingError) / Util.DT;
        double dTermDps = headingKD.getAsDouble() * errorRate;
        omegaDps += dTermDps;
        previousHeadingError = headingError;

        omegaDps = MathUtil.clamp(omegaDps, -maxOmegaDps, maxOmegaDps);

        // add manual trim
        double rotTrim = rotInput * maxRotationRps * rotationTrimAuthority.getAsDouble();
        double omega = Math.toRadians(omegaDps) + rotTrim;

        // drive with rotation preserved
        drivePreserveRotation(vxField, vyField, omega, maxSpeedMps);
    }

    /**
     * Drives with rotation-priority desaturation.
     * <p>
     * When wheel speeds would exceed limits, scales down translation
     * to preserve rotation authority.
     */
    private void drivePreserveRotation(double vx, double vy, double omega, double maxSpeed) {
        // convert field-relative to robot-relative
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotSpeeds = Util.fromDriverRelativeSpeeds(fieldSpeeds, swerve.getHeading());

        // get module states
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(robotSpeeds);

        // find max wheel speed
        double maxWheelSpeed = 0;
        for (SwerveModuleState state : states) {
            maxWheelSpeed = Math.max(maxWheelSpeed, Math.abs(state.speedMetersPerSecond));
        }

        // if within limits, just drive normally
        if (maxWheelSpeed <= maxSpeed) {
            swerve.driveStates("orbit", states, robotSpeeds);
            return;
        }

        // calculate how much rotation alone would use
        ChassisSpeeds rotationOnly = new ChassisSpeeds(0, 0, omega);
        SwerveModuleState[] rotationStates = KINEMATICS.toSwerveModuleStates(rotationOnly);
        double maxRotationSpeed = 0;
        for (SwerveModuleState state : rotationStates) {
            maxRotationSpeed = Math.max(maxRotationSpeed, Math.abs(state.speedMetersPerSecond));
        }

        // calculate available headroom for translation
        double availableForTranslation = maxSpeed - maxRotationSpeed;
        if (availableForTranslation <= 0) {
            // rotation alone exceeds max - scale rotation and skip translation
            double scale = maxSpeed / maxRotationSpeed;
            robotSpeeds = new ChassisSpeeds(0, 0, omega * scale);
        } else {
            // scale translation to fit in remaining headroom
            double translationMag = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
            if (translationMag > 0) {
                double scale = Math.min(1.0, availableForTranslation / translationMag);
                robotSpeeds = new ChassisSpeeds(
                    robotSpeeds.vxMetersPerSecond * scale,
                    robotSpeeds.vyMetersPerSecond * scale,
                    omega
                );
            }
        }

        states = KINEMATICS.toSwerveModuleStates(robotSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
        swerve.driveStates("orbit", states, robotSpeeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Trapezoid;
import frc.robot.util.Util;

import java.util.Objects;

import static frc.robot.Config.SwerveAuto.maxRotationAcceleration;
import static frc.robot.Config.SwerveAuto.maxRotationVelocity;
import static frc.robot.Config.SwerveAuto.maxTranslationAcceleration;
import static frc.robot.Config.SwerveAuto.maxTranslationVelocity;
import static frc.robot.Config.SwerveAuto.rotationKp;
import static frc.robot.Config.SwerveAuto.translationKp;

/**
 * Automatically drives the robot to a target pose. This can include both
 * translation and rotation.
 * <ul>
 *
 *     <li>If translation is required, we will drive along a straight
 *     line between the two poses, with a trapezoid profile to ensure a
 *     smooth-in, smooth-out motion;</li>
 *
 *     <li>If rotation is required, we will spin the robot, also using a
 *     trapezoid profile for the same reason.</li>
 *
 * </ul>
 *
 * This command is time-based: it will calculated expected runtime and stop
 * after that many seconds, regardless of whether it achieved the desired
 * pose. If it's mis-tuned or the robot gets bumped, results may vary.
 */
public class SwerveToPoseCommand2 extends Command {

    // hardcoded limits for minimum distance and angle of movement
    static final double MIN_DISTANCE = Units.degreesToRadians(3.0);
    static final double MIN_ANGLE = Units.inchesToMeters(3.0);

    static final boolean VERBOSE = true;

    final SwerveSubsystem swerve;
    final Pose2d finalPose;
    final Trapezoid translationTrapezoid;
    final Trapezoid rotationTrapezoid;
    final PIDController pidX;
    final PIDController pidY;
    final PIDController pidOmega;
    final Timer timer;
    Pose2d startPose;
    double meters;
    double radians;
    double totalTime;
    boolean translationRequired;
    boolean rotationRequired;
    boolean running;
    double cos;
    double sin;

    /**
     * Creates a {@link SwerveToPoseCommand}
     * @param swerve the swerve drive
     * @param finalPose the target pose
     */
    public SwerveToPoseCommand2(SwerveSubsystem swerve, Pose2d finalPose) {
        this.swerve = Objects.requireNonNull(swerve);
        this.finalPose = Objects.requireNonNull(finalPose);
        this.translationTrapezoid = new Trapezoid(
                () -> Units.feetToMeters(maxTranslationVelocity.getAsDouble()),
                () -> Units.feetToMeters(maxTranslationAcceleration.getAsDouble()));
        this.rotationTrapezoid = new Trapezoid(
                () -> Units.degreesToRadians(maxRotationVelocity.getAsDouble()),
                () -> Units.degreesToRadians(maxRotationAcceleration.getAsDouble()));
        this.pidX = new PIDController(translationKp.getAsDouble(), 0.0, 0.0);
        this.pidY = new PIDController(translationKp.getAsDouble(), 0.0, 0.0);
        this.pidOmega = new PIDController(rotationKp.getAsDouble(), 0.0, 0.0);
        this.timer = new Timer();
    }

    /**
     * Initializes our path calculation
     */
    @Override
    public void initialize() {

        // calculate start pose
        startPose = swerve.getPose();

        // we implement translation by calculating an offset from the start
        // pose which moves along a straight line towards the final pose. we
        // will ignore translations that are too small.
        meters = Util.metersBetween(startPose, finalPose);
        if (MathUtil.isNear(0.0, meters, MIN_DISTANCE)) {
            translationRequired = false;
        } else {
            translationRequired = true;
            translationTrapezoid.calculate(0.0, meters);
            totalTime = Math.max(totalTime, translationTrapezoid.totalTime());
        }

        // if we're translating, we need to know the angle of the line between
        // the start and final poses; the cos and sin of this angle will let us
        // decompose straight-line movement into separate X and Y components
        if (translationRequired) {
            Rotation2d translationAngle = finalPose.getTranslation()
                    .minus(startPose.getTranslation())
                    .getAngle();
            cos = translationAngle.getCos();
            sin = translationAngle.getSin();
        } else {
            cos = 0.0;
            sin = 0.0;
        }

        if (VERBOSE) {
            Util.log("=================================================");
            Util.log("[auto-pose] tx speed = %.2f", maxTranslationVelocity.getAsDouble());
            Util.log("[auto-pose] rx speed = %.2f", maxRotationVelocity.getAsDouble());
            Util.log("[auto-pose] meters = %.2f", meters);
            Util.log("[auto-pose] radians = %.2f", radians);
            Util.log("[auto-pose] sin = %.2f", sin);
            Util.log("[auto-pose] cos = %.2f", cos);
            Util.log("[auto-pose] totalTime = %.2f", totalTime);
            Util.log("=================================================");
        }

        // we implement rotation by calculating an angle offset from the start
        // heading which moves towards the final heading. we ignore motions that
        // are too small
        radians = Util.radiansBetween(startPose, finalPose);
        if (MathUtil.isNear(0.0, radians, MIN_ANGLE)) {
            rotationRequired = false;
        } else {
            rotationRequired = true;
            rotationTrapezoid.calculate(0.0, radians);
            totalTime = Math.max(totalTime, rotationTrapezoid.totalTime());
        }

        running = (translationRequired || rotationRequired);
        if (running) {
            Util.log("[auto-pose] driving %.2f feet and turning %.2f degrees",
                    Units.metersToFeet(meters),
                    Units.radiansToDegrees(radians));
            resetPid(pidX, translationKp.getAsDouble());
            resetPid(pidY, translationKp.getAsDouble());
            resetPid(pidOmega, rotationKp.getAsDouble());
            timer.restart();
        } else {
            Util.log("[auto-pose] nothing to do");
        }
    }

    /**
     * Reset the P parameter and calculated errors
     */
    private void resetPid(PIDController pid, double p) {
        pid.setP(p);
        pid.reset();
    }

    /**
     * Calculate the desired {@link ChassisSpeeds} to
     */
    @Override
    public void execute() {

        // if there's nothing to do, then do nothing
        if (!running) {
            return;
        }

        double t = timer.get();
        double speedX = 0.0;
        double speedY = 0.0;
        double speedOmega = 0.0;
        double poseX = startPose.getX();
        double poseY = startPose.getY();
        double poseOmega = startPose.getRotation().getRadians();

        // if we're rotating, we get the state of the rotation profile. the
        // position will be the offset from the start heading at this moment
        // in time.
        if (rotationRequired) {
            State state = rotationTrapezoid.sample(t);
            poseOmega = MathUtil.angleModulus(poseOmega + state.position);
            speedOmega = state.velocity;
        }

        // if we're translating, we get the state of the translation profile.
        // the position and velocity will along the line from start to finish;
        // we decompose them along X/Y using cos/sin.
        if (translationRequired) {
            State state = translationTrapezoid.sample(t);
            speedX = state.velocity * cos;
            speedY = state.velocity * sin;
            poseX += state.position * cos;
            poseY += state.position * sin;
        }

        // based on where we are, where we want to be, and how fast we think
        // we need to be going, let's calculate our final speed
        Pose2d currentPose = swerve.getPose();
        Pose2d desiredPose = new Pose2d(poseX, poseY, Rotation2d.fromRadians(poseOmega));
        ChassisSpeeds desiredSpeeds = addFeedbackAndClamp(
                currentPose,
                desiredPose,
                speedX, speedY, speedOmega);

        // since these calculations give us field-relative speeds, we need to
        // translate them into robot-relative speeds so we can tell the robot
        // to drive the intended path
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredSpeeds,
                currentPose.getRotation());

        if (VERBOSE) {
            Util.log("[auto-pose] speed @ %.2f = %s", t, robotRelativeSpeeds);
            SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", true);
            SmartDashboard.putNumber("SwerveAutoPoseCommand/SpeedX", Units.metersToFeet(robotRelativeSpeeds.vxMetersPerSecond));
            SmartDashboard.putNumber("SwerveAutoPoseCommand/SpeedY", Units.metersToFeet(robotRelativeSpeeds.vyMetersPerSecond));
            SmartDashboard.putNumber("SwerveAutoPoseCommand/SpeedOmega", Math.toDegrees(robotRelativeSpeeds.omegaRadiansPerSecond));
            SmartDashboard.putNumber("SwerveAutoPoseCommand/ErrorX", pidX.getError());
            SmartDashboard.putNumber("SwerveAutoPoseCommand/ErrorY", pidY.getError());
            SmartDashboard.putNumber("SwerveAutoPoseCommand/ErrorOmega", pidOmega.getError());
            Util.publishPose("AutoPoseNext", desiredPose);
            Util.publishPose("AutoPoseFinal", finalPose);
        }

        swerve.driveRobotRelative("auto", robotRelativeSpeeds);
    }

    /**
     * Applies PID feedback and clamping to desired chassis speeds
     */
    private ChassisSpeeds addFeedbackAndClamp(Pose2d currentPose, Pose2d desiredPose, double speedX, double speedY, double speedOmega) {

        // tweak desired speeds using PID to correct for position inaccuracy
        speedX += pidX.calculate(currentPose.getX(), desiredPose.getX());
        speedY += pidY.calculate(currentPose.getY(), desiredPose.getY());
        speedOmega += pidOmega.calculate(
                currentPose.getRotation().getDegrees(),
                desiredPose.getRotation().getDegrees());

        // calculate target speed in feet per second; if it's >max we will
        // want to scale the x and y speeds down
        double mt = maxTranslationVelocity.getAsDouble();
        double fps = Units.metersToFeet(Math.hypot(speedX, speedY));
        if (fps > mt) {
            double factor = mt / fps;
            speedX *= factor;
            speedY *= factor;
        }

        // clamp the speed (making sure to translate the max value into
        // radians b/c that's what's in ChassisSpeeds
        speedOmega = Util.applyClamp(
                speedOmega,
                () -> Units.degreesToRadians(maxRotationVelocity.getAsDouble()));

        return new ChassisSpeeds(speedX, speedY, speedOmega);
    }

    /**
     * Since we direct motion using a timed profile, we base our decision
     * of "doneness" on time rather than position. that way, if we get
     * bumped or are mis-tuned, the command will quit instead of waiting
     * forever to get to a position it will never reach.
     */
    @Override
    public boolean isFinished() {
        return !running || timer.hasElapsed(totalTime);
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
        SmartDashboard.putBoolean("SwerveToPoseCommand/Running?", false);
    }
}

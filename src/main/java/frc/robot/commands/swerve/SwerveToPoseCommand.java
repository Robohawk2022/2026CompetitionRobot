package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Trapezoid;
import frc.robot.util.Util;

import static frc.robot.Config.SwerveAuto.*;

/**
 * Command that drives the swerve to a target pose along a straight line. Uses
 * two independent {@link Trapezoid} profiles:
 * <ul>
 *   <li>Translation: controls speed along the straight-line path to target</li>
 *   <li>Rotation: controls angular velocity to target heading</li>
 * </ul>
 */
public class SwerveToPoseCommand extends Command {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    final SwerveSubsystem swerve;
    final Pose2d targetPose;
    final Trapezoid translationTrapezoid;
    final Trapezoid rotationTrapezoid;
    final Timer timer;

    // when we initialize, we capture the starting pose and calculate the
    // distance and direction to our goal
    Pose2d startPose;
    Translation2d directionUnit;
    double distanceGoalFeet;
    double headingGoalDeg;

    // distance calculation: we start at 0 distance, and translate towards the
    // goal. at each step, we have a "next" desired distance and velocity,
    // and we calculate error and feedback against that.
    double distanceCurrentFeet;
    double distanceDesiredFeet;
    double velocityDesiredFps;
    double distanceErrorFeet;
    double distanceFeedbackFps;

    // heading calculation: we start at a certain heading, and rotate towards
    // the goal. at each step, we have a "next" desired heading and velocity,
    // and we calculate error and feedback against that.
    double headingStartDeg;
    double headingCurrentDeg;
    double headingDesiredDeg;
    double omegaDesiredDps;
    double headingErrorDeg;
    double headingFeedbackDps;

    /**
     * Creates a {@link SwerveToPoseCommand}.
     *
     * @param swerve the swerve subsystem (required)
     * @param targetPose the target pose to drive to (required)
     */
    public SwerveToPoseCommand(SwerveSubsystem swerve, Pose2d targetPose) {

        this.swerve = Objects.requireNonNull(swerve);
        this.targetPose = Objects.requireNonNull(targetPose);
        this.translationTrapezoid = new Trapezoid(maxTranslationVelocity, maxTranslationAcceleration);
        this.rotationTrapezoid = new Trapezoid(maxRotationVelocity, maxRotationAcceleration);
        this.timer = new Timer();

        addRequirements(swerve);

        if (verboseLogging) {
            SmartDashboard.putData("SwerveToPoseCommand", builder -> {

                builder.addBooleanProperty("Running?", this::isScheduled, null);

                // Distance* groups together: Current, Desired, Error, Goal
                builder.addDoubleProperty("DistanceCurrent", () -> distanceCurrentFeet, null);
                builder.addDoubleProperty("DistanceDesired", () -> distanceDesiredFeet, null);
                builder.addDoubleProperty("DistanceError", () -> distanceErrorFeet, null);
                builder.addDoubleProperty("DistanceGoal", () -> distanceGoalFeet, null);

                // Heading* groups together: Current, Desired, Error, Goal
                builder.addDoubleProperty("HeadingCurrent", () -> headingCurrentDeg, null);
                builder.addDoubleProperty("HeadingDesired", () -> headingDesiredDeg, null);
                builder.addDoubleProperty("HeadingError", () -> headingErrorDeg, null);
                builder.addDoubleProperty("HeadingGoal", () -> headingGoalDeg, null);

                // Velocity* groups together (translation): Desired, Feedback
                builder.addDoubleProperty("VelocityDesired", () -> velocityDesiredFps, null);
                builder.addDoubleProperty("VelocityFeedback", () -> distanceFeedbackFps, null);

                // Omega* groups together (rotation): Desired, Feedback
                builder.addDoubleProperty("OmegaDesired", () -> omegaDesiredDps, null);
                builder.addDoubleProperty("OmegaFeedback", () -> headingFeedbackDps, null);
            });
        }
    }

    @Override
    public void initialize() {

        startPose = swerve.getPose();

        // calculate translation path
        Translation2d delta = targetPose.getTranslation().minus(startPose.getTranslation());
        distanceGoalFeet = Units.metersToFeet(delta.getNorm());

        // calculate unit vector for direction (handle zero distance case)
        if (distanceGoalFeet > 0.001) {
            directionUnit = delta.div(delta.getNorm());
        } else {
            directionUnit = Translation2d.kZero;
        }

        // calculate rotation path (shortest path)
        headingStartDeg = swerve.getHeading().getDegrees();
        headingGoalDeg = targetPose.getRotation().getDegrees();
        double headingError = Util.degreeModulus(headingGoalDeg - headingStartDeg);
        headingGoalDeg = headingStartDeg + headingError;

        // configure profiles
        translationTrapezoid.calculate(0.0, distanceGoalFeet);
        rotationTrapezoid.calculate(headingStartDeg, headingGoalDeg);
        timer.restart();

        Util.log("[auto-pose] driving to pose: %.1f ft translation, %.1f deg rotation",
                distanceGoalFeet,
                headingError);
    }

    @Override
    public void execute() {

        double t = timer.get();

        // sample translation profile
        State translationState = translationTrapezoid.sample(t);
        distanceDesiredFeet = translationState.position;
        velocityDesiredFps = translationState.velocity;

        // sample rotation profile
        State rotationState = rotationTrapezoid.sample(t);
        headingDesiredDeg = rotationState.position;
        omegaDesiredDps = rotationState.velocity;

        // update current state for dashboard and error calculation
        distanceCurrentFeet = Util.feetBetween(swerve.getPose(), targetPose);
        headingCurrentDeg = swerve.getHeading().getDegrees();

        // calculate errors
        distanceErrorFeet = distanceCurrentFeet;
        headingErrorDeg = Util.degreeModulus(headingGoalDeg - headingCurrentDeg);

        // feedforward from profile + proportional feedback on error
        distanceFeedbackFps = translationKp.getAsDouble() * distanceErrorFeet;
        headingFeedbackDps = rotationKp.getAsDouble() * headingErrorDeg;

        double velocityFps = velocityDesiredFps + distanceFeedbackFps;
        double omegaDps = omegaDesiredDps + headingFeedbackDps;

        // calculate field-relative velocity from profile
        // velocity along the path times the unit direction vector
        double velocityMps = Units.feetToMeters(velocityFps);
        double vx = velocityMps * directionUnit.getX();
        double vy = velocityMps * directionUnit.getY();
        double omega = Math.toRadians(omegaDps);

        // drive field-relative speeds converted to robot-relative
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldSpeeds,
                swerve.getHeading());
        swerve.driveRobotRelative("auto-pose", robotSpeeds);
    }

    @Override
    public boolean isFinished() {

        // not finished until both profiles are complete
        double t = timer.get();
        if (t < translationTrapezoid.totalTime() || t < rotationTrapezoid.totalTime()) {
            return false;
        }
        return true;

        // // check position tolerance
        // double distanceError = Util.feetBetween(swerve.getPose(), targetPose);
        // if (distanceError > positionTolerance.getAsDouble()) {
        //     return false;
        // }

        // // check heading tolerance
        // double headingError = Math.abs(Util.degreeModulus(
        //         targetPose.getRotation().getDegrees() - swerve.getHeading().getDegrees()));
        // return headingError < headingTolerance.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotRelative("idle", Util.ZERO_SPEED);
        if (interrupted) {
            Util.log("[auto-pose] pose command interrupted");
        } else {
            Util.log("[auto-pose] pose command complete at (%.1f, %.1f,%.1f)",
                    Units.metersToFeet(swerve.getPose().getX()),
                    Units.metersToFeet(swerve.getPose().getY()),
                    swerve.getHeading().getDegrees());
        }
        timer.stop();
    }
}

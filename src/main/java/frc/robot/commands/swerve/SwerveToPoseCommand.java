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
 * Command that drives the swerve to a target pose along a straight line using
 * trapezoidal motion profiling for both translation and rotation.
 * <p>
 * Uses two independent {@link Trapezoid} profiles:
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

    // computed on initialize
    Pose2d startPose;
    Translation2d directionUnit;
    double totalDistanceFeet;
    double startHeadingDeg;
    double targetHeadingDeg;

    // updated each execute for dashboard
    double currentDistanceFeet;
    double currentHeadingDeg;
    double profileDistanceFeet;
    double profileVelocityFps;
    double profileHeadingDeg;
    double profileOmegaDps;

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
                builder.addDoubleProperty("TotalDistance", () -> totalDistanceFeet, null);
                builder.addDoubleProperty("CurrentDistance", () -> currentDistanceFeet, null);
                builder.addDoubleProperty("ProfileDistance", () -> profileDistanceFeet, null);
                builder.addDoubleProperty("ProfileVelocity", () -> profileVelocityFps, null);
                builder.addDoubleProperty("TargetHeading", () -> targetHeadingDeg, null);
                builder.addDoubleProperty("CurrentHeading", () -> currentHeadingDeg, null);
                builder.addDoubleProperty("ProfileHeading", () -> profileHeadingDeg, null);
                builder.addDoubleProperty("ProfileOmega", () -> profileOmegaDps, null);
            });
        }
    }

    @Override
    public void initialize() {

        startPose = swerve.getPose();

        // calculate translation path
        Translation2d delta = targetPose.getTranslation().minus(startPose.getTranslation());
        totalDistanceFeet = Units.metersToFeet(delta.getNorm());

        // calculate unit vector for direction (handle zero distance case)
        if (totalDistanceFeet > 0.001) {
            directionUnit = delta.div(delta.getNorm());
        } else {
            directionUnit = Translation2d.kZero;
        }

        // calculate rotation path (shortest path)
        startHeadingDeg = swerve.getHeading().getDegrees();
        targetHeadingDeg = targetPose.getRotation().getDegrees();
        double headingError = Util.degreeModulus(targetHeadingDeg - startHeadingDeg);
        targetHeadingDeg = startHeadingDeg + headingError;

        // configure profiles
        translationTrapezoid.calculate(0.0, totalDistanceFeet);
        rotationTrapezoid.calculate(startHeadingDeg, targetHeadingDeg);
        timer.restart();

        Util.log("[swerve] driving to pose: %.1f ft, %.1f deg rotation",
                totalDistanceFeet,
                headingError);
    }

    @Override
    public void execute() {

        double t = timer.get();

        // sample translation profile
        State translationState = translationTrapezoid.sample(t);
        profileDistanceFeet = translationState.position;
        profileVelocityFps = translationState.velocity;

        // sample rotation profile
        State rotationState = rotationTrapezoid.sample(t);
        profileHeadingDeg = rotationState.position;
        profileOmegaDps = rotationState.velocity;

        // calculate field-relative velocity from profile
        // velocity along the path times the unit direction vector
        double velocityMps = Units.feetToMeters(profileVelocityFps);
        double vx = velocityMps * directionUnit.getX();
        double vy = velocityMps * directionUnit.getY();
        double omega = Math.toRadians(profileOmegaDps);

        // update current state for dashboard
        currentDistanceFeet = Util.feetBetween(swerve.getPose(), targetPose);
        currentHeadingDeg = swerve.getHeading().getDegrees();

        // drive field-relative speeds converted to robot-relative
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldSpeeds,
                swerve.getHeading());
        swerve.driveRobotRelative("toPose", robotSpeeds);
    }

    @Override
    public boolean isFinished() {

        // not finished until both profiles are complete
        double t = timer.get();
        if (t < translationTrapezoid.totalTime() || t < rotationTrapezoid.totalTime()) {
            return false;
        }

        // check position tolerance
        double distanceError = Util.feetBetween(swerve.getPose(), targetPose);
        if (distanceError > positionTolerance.getAsDouble()) {
            return false;
        }

        // check heading tolerance
        double headingError = Math.abs(Util.degreeModulus(
                targetPose.getRotation().getDegrees() - swerve.getHeading().getDegrees()));
        return headingError < headingTolerance.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotRelative("idle", Util.ZERO_SPEED);
        if (interrupted) {
            Util.log("[swerve] pose command interrupted");
        } else {
            Util.log("[swerve] pose command complete at (%.1f, %.1f) heading %.1f",
                    Units.metersToFeet(swerve.getPose().getX()),
                    Units.metersToFeet(swerve.getPose().getY()),
                    swerve.getHeading().getDegrees());
        }
    }
}

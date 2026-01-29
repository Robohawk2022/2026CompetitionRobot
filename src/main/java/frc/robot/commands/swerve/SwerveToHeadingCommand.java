package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Trapezoid;
import frc.robot.util.Util;

import static frc.robot.Config.SwerveAuto.*;

/**
 * Command that rotates the swerve drive to face a target heading. Uses a
 * {@link Trapezoid} profile to smoothly accelerate and decelerate the rotation,
 * controlled by parameters in {@code Config.SwerveAuto}.
 */
public class SwerveToHeadingCommand extends Command {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    final SwerveSubsystem swerve;
    final Rotation2d targetHeading;
    final Trapezoid trapezoid;
    final Timer timer;

    // we start at a certain heading, and rotate towards the goal. at each
    // step, we have a "next" desired heading and velocity, and we calculate
    // error and feedback against that.
    double headingStartDeg;
    double headingGoalDeg;
    double headingCurrentDeg;
    double headingDesiredDeg;
    double velocityDesiredDeg;
    double headingErrorDeg;
    double feedbackDps;

    /**
     * Creates a {@link SwerveToHeadingCommand}.
     *
     * @param swerve the swerve subsystem (required)
     * @param targetHeading the target heading to rotate to (required)
     */
    public SwerveToHeadingCommand(SwerveSubsystem swerve, Rotation2d targetHeading) {

        this.swerve = Objects.requireNonNull(swerve);
        this.targetHeading = Objects.requireNonNull(targetHeading);
        this.trapezoid = new Trapezoid(maxRotationVelocity, maxRotationAcceleration);
        this.timer = new Timer();

        addRequirements(swerve);

        if (verboseLogging) {
            SmartDashboard.putData("SwerveToHeadingCommand", builder -> {
                builder.addBooleanProperty("Running?", this::isScheduled, null);

                // Heading* groups together: Current, Desired, Error, Goal, Start
                builder.addDoubleProperty("HeadingStart", () -> headingStartDeg, null);
                builder.addDoubleProperty("HeadingGoal", () -> headingGoalDeg, null);
                builder.addDoubleProperty("HeadingCurrent", () -> headingCurrentDeg, null);
                builder.addDoubleProperty("HeadingDesired", () -> headingDesiredDeg, null);
                builder.addDoubleProperty("HeadingError", () -> headingErrorDeg, null);

                // Velocity* groups together: Desired, Feedback
                builder.addDoubleProperty("VelocityDesired", () -> velocityDesiredDeg, null);
                builder.addDoubleProperty("VelocityFeedback", () -> feedbackDps, null);
            });
        }
    }

    @Override
    public void initialize() {

        // get current heading and calculate shortest path to target
        headingStartDeg = swerve.getHeading().getDegrees();
        headingGoalDeg = targetHeading.getDegrees();

        // calculate the shortest rotation path (-180 to 180)
        double error = Util.degreeModulus(headingGoalDeg - headingStartDeg);
        headingGoalDeg = headingStartDeg + error;

        // configure the trapezoid profile
        trapezoid.calculate(headingStartDeg, headingGoalDeg);
        timer.restart();

        Util.log("[auto-heading] rotating from %.1f to %.1f (%.1f deg)",
                headingStartDeg,
                headingGoalDeg,
                error);
    }

    @Override
    public void execute() {

        // sample the profile at the current time
        State state = trapezoid.sample(timer.get());
        headingDesiredDeg = state.position;
        velocityDesiredDeg = state.velocity;

        // get current heading and calculate error
        headingCurrentDeg = swerve.getHeading().getDegrees();
        headingErrorDeg = Util.degreeModulus(headingGoalDeg - headingCurrentDeg);

        // feedforward from profile + proportional feedback on error
        feedbackDps = rotationKp.getAsDouble() * headingErrorDeg;
        double omegaDps = velocityDesiredDeg + feedbackDps;

        // convert velocity to radians/sec and drive
        double omegaRadPerSec = Math.toRadians(omegaDps);
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omegaRadPerSec);
        swerve.driveRobotRelative("auto-heading", speeds);
    }

    @Override
    public boolean isFinished() {
        // finished when profile is complete and we're within tolerance
        if (timer.get() < trapezoid.totalTime()) {
            return false;
        }
        double error = Math.abs(Util.degreeModulus(
                targetHeading.getDegrees() - swerve.getHeading().getDegrees()));
        return error < headingTolerance.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotRelative("idle", Util.ZERO_SPEED);
        if (interrupted) {
            Util.log("[auto-heading] heading command interrupted");
        } else {
            Util.log("[auto-heading] heading command complete at %.1f deg",
                    swerve.getHeading().getDegrees());
        }
        timer.stop();
    }
}

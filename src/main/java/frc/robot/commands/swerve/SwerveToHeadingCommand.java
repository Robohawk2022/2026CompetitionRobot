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
 * Command that rotates the swerve drive to face a target heading using
 * trapezoidal motion profiling.
 * <p>
 * The command uses a {@link Trapezoid} profile to smoothly accelerate and
 * decelerate the rotation, controlled by parameters in {@code Config.SwerveAuto}.
 */
public class SwerveToHeadingCommand extends Command {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    final SwerveSubsystem swerve;
    final Rotation2d targetHeading;
    final Trapezoid trapezoid;
    final Timer timer;

    double startHeadingDeg;
    double targetHeadingDeg;
    double currentHeadingDeg;
    double profilePositionDeg;
    double profileVelocityDps;

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
                builder.addDoubleProperty("StartHeading", () -> startHeadingDeg, null);
                builder.addDoubleProperty("TargetHeading", () -> targetHeadingDeg, null);
                builder.addDoubleProperty("CurrentHeading", () -> currentHeadingDeg, null);
                builder.addDoubleProperty("ProfilePosition", () -> profilePositionDeg, null);
                builder.addDoubleProperty("ProfileVelocity", () -> profileVelocityDps, null);
            });
        }
    }

    @Override
    public void initialize() {

        // get current heading and calculate shortest path to target
        startHeadingDeg = swerve.getHeading().getDegrees();
        targetHeadingDeg = targetHeading.getDegrees();

        // calculate the shortest rotation path (-180 to 180)
        double error = Util.degreeModulus(targetHeadingDeg - startHeadingDeg);
        targetHeadingDeg = startHeadingDeg + error;

        // configure the trapezoid profile
        trapezoid.calculate(startHeadingDeg, targetHeadingDeg);
        timer.restart();

        Util.log("[swerve] rotating from %.1f to %.1f (%.1f deg)",
                startHeadingDeg,
                targetHeadingDeg,
                error);
    }

    @Override
    public void execute() {

        // sample the profile at the current time
        State state = trapezoid.sample(timer.get());
        profilePositionDeg = state.position;
        profileVelocityDps = state.velocity;

        // get current heading for logging
        currentHeadingDeg = swerve.getHeading().getDegrees();

        // convert velocity to radians/sec and drive
        double omegaRadPerSec = Math.toRadians(profileVelocityDps);
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omegaRadPerSec);
        swerve.driveRobotRelative("toHeading", speeds);
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
            Util.log("[swerve] heading command interrupted");
        } else {
            Util.log("[swerve] heading command complete at %.1f deg",
                    swerve.getHeading().getDegrees());
        }
    }
}

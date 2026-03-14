package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.Util;

import static frc.robot.Config.BallHandling.shootAngleTolerance;
import static frc.robot.Config.BallHandling.shootDistanceFeet;
import static frc.robot.Config.BallHandling.shootDistanceTolerance;
import static frc.robot.Config.SwerveAuto.openHopperSecs;
import static frc.robot.Config.SwerveAuto.openHopperVelocity;

/**
 * Factory methods for shooting command sequences.
 */
public class ShootingCommands {

    public static final boolean VERBOSE = true;

    /**
     * Spin up the shooter to intake speed then run the ball path in
     * intake mode
     */
    public static Command intakeMode(LEDSubsystem led, BallPathSubsystem ballPath, ShooterSubsystem shooter) {

        // the LED and shooter commands will run forever. the last one will
        // pause to let the shooter change target speeds, and then quit when
        // it's at the new target. the whole thing ends when the shooter is
        // at speed.
        Command step1 = Commands.race(
                led.flash(LEDSignal.SPINNING_UP),
                shooter.intakeCommand().until(shooter::atSpeed),
                Commands.waitSeconds(0.5).andThen(Commands.waitUntil(shooter::atSpeed)));

        Command step2 = Commands.parallel(
                led.flash(LEDSignal.INTAKING),
                shooter.intakeCommand(),
                ballPath.intakeCommand());

        return step1.andThen(step2);
    }

    /**
     * Spin up the shooter to shoot speed then run the ball path in
     * feed mode
     */
    public static Command shootMode(LEDSubsystem led, BallPathSubsystem ballPath, ShooterSubsystem shooter) {

        // the LED and shooter commands will run forever. the last one will
        // pause to let the shooter change target speeds, and then quit when
        // it's at the new target. the whole thing ends when the shooter is
        // at speed.
        Command step1 = Commands.race(
                led.flash(LEDSignal.SPINNING_UP),
                shooter.shootCommand().until(shooter::atSpeed),
                Commands.waitSeconds(2.0).andThen(Commands.waitUntil(shooter::atSpeed)));

        Command step2 = Commands.parallel(
                led.show(LEDSignal.SHOOTING),
                shooter.shootCommand(),
                ballPath.feedCommand());

        return step1.andThen(step2);
    }

    /**
     * Rapidly oscillates the robot left and right (robot-relative) to
     * dislodge stuck balls. Hold trigger to jiggle, release to stop.
     *
     * @return the jiggle command
     */
    public static Command jiggle(SwerveSubsystem swerve) {

        // ~1-2 inches of travel at ~6 Hz oscillation
        double speedMps = Units.feetToMeters(2.0); // 2 ft/s sideways

        // full cycle = 0.16s (~6 Hz)
        double periodSec = 0.16;
        double halfPeriod = periodSec / 2.0;

        Timer timer = new Timer();
        return swerve.startRun(
                timer::restart,
                () -> {
                    // alternate left/right each half-period
                    double t = timer.get() % periodSec;
                    double vy = (t < halfPeriod) ? speedMps : -speedMps;
                    swerve.driveRobotRelative("jiggle", new ChassisSpeeds(0, vy, 0));
                }
        );
    }

    /**
     * @return command to run our ghetto "open the hopper" routine
     */
    public static Command openHopper(SwerveSubsystem swerve) {
        return swerve.defer(() -> {
            double vel = openHopperVelocity.getAsDouble();
            double secs = openHopperSecs.getAsDouble();
            Command back = swerve.driveAtSpeedCommand("hop-b",
                    new ChassisSpeeds(-vel, 0.0, 0.0));
            Command fwd = swerve.driveAtSpeedCommand("hop-f",
                    new ChassisSpeeds(vel, 0.0, 0.0));
            return back.withTimeout(secs).andThen(fwd.withTimeout(secs));
        });
    }

    /**
     * @return a command that to drive the
     * robot into a shooting position in one fell swoop
     */
    public static Command orientToShoot(LEDSubsystem led, SwerveSubsystem swerve) {

        return swerve.defer(() -> {

            // compute target pose: farDistance feet from hub, on the line
            // between robot and hub, facing the hub
            Pose2d hubCenter = Field.getHubCenter();
            Pose2d robotPose = swerve.getPose();

            // direction from hub toward robot (unit vector)
            Translation2d hubToRobot = robotPose.getTranslation().minus(hubCenter.getTranslation());
            double distMeters = hubToRobot.getNorm();

            Translation2d direction;
            if (distMeters > 0.01) {
                direction = hubToRobot.div(distMeters);
            } else {
                // robot is on top of hub, pick an arbitrary direction
                direction = new Translation2d(1.0, 0.0);
            }

            // target position: hub center + direction * farDistance
            double farDistMeters = Units.feetToMeters(shootDistanceFeet.getAsDouble());
            Translation2d targetTranslation = hubCenter.getTranslation()
                    .plus(direction.times(farDistMeters));

            // target heading: face the hub (opposite of direction from hub)
            Rotation2d targetHeading = direction.times(1.0).getAngle();

            return Commands.parallel(
                    led.flash(LEDSignal.AIMING),
                    swerve.driveToPoseCommand(new Pose2d(targetTranslation, targetHeading)));
        });
    }

    /**
     * @return true if the current pose of the robot has it (a) facing the
     * center of the hub within
     */
    public static boolean isInShootingPosition(SwerveSubsystem swerve) {

        Pose2d hubCenter = Field.getHubCenter();
        Pose2d robotPose = swerve.getPose();

        // current angle and distance to the hub
        double distanceCurrent = Util.feetBetween(hubCenter, robotPose);
        double distanceTarget = shootDistanceFeet.getAsDouble();
        double distanceTolerance = shootDistanceTolerance.getAsDouble();

        double angleCurrent = hubCenter.getTranslation()
                .minus(robotPose.getTranslation())
                .getAngle().minus(robotPose.getRotation())
                .getDegrees();
        double angleTarget = Rotation2d.k180deg.getDegrees();
        double angleTolerance = shootAngleTolerance.getAsDouble();

        if (VERBOSE) {
            Util.publishPose("TargetingHub", hubCenter);
            SmartDashboard.putNumber("Targeting/DistanceCurrent", distanceCurrent);
            SmartDashboard.putNumber("Targeting/DistanceTarget", distanceTarget);
            SmartDashboard.putNumber("Targeting/DistanceTolerance", distanceTolerance);
            SmartDashboard.putNumber("Targeting/AngleCurrent", angleCurrent);
            SmartDashboard.putNumber("Targeting/AngleTarget", angleTarget);
            SmartDashboard.putNumber("Targeting/AngleTolerance", angleTolerance);
        }

        // if we're too far from the hub, we're not in position
        return MathUtil.isNear(distanceTarget, distanceCurrent, distanceTolerance)
            && MathUtil.isNear(angleTarget, angleCurrent, angleTolerance, -180.0, 180.0);
    }
}

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameController;
import frc.robot.commands.swerve.SwerveOrbitCommand;
import frc.robot.commands.swerve.SwerveToHeadingCommand;
import frc.robot.commands.swerve.SwerveToPoseCommand;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.Util;

import static frc.robot.Config.Shooter.shootDistanceFeet;
import static frc.robot.Config.SwerveAuto.openHopperSecs;
import static frc.robot.Config.SwerveAuto.openHopperVelocity;

/**
 * Factory methods for shooting command sequences.
 */
public class ShootingCommands {

    /**
     * Spin up the shooter to intake speed then run the ball path in
     * intake mode
     */
    public static Command intakeMode(LEDSubsystem led, BallPathSubsystem ballPath, ShooterSubsystem shooter) {

        Command step1 = Commands.parallel(
                led.flash(LEDSignal.SPINNING_UP),
                shooter.intakeCommand().until(shooter::atSpeed));

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

        Command step1 = Commands.parallel(
                led.flash(LEDSignal.SPINNING_UP),
                shooter.shootCommand().until(shooter::atSpeed));

        Command step2 = Commands.parallel(
                led.flash(LEDSignal.CELEBRATION),
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
            return back.withTimeout(secs).andThen(fwd.withTimeout(2.0 * secs));
        });
    }

    /**
     * @return a command that uses {@link SwerveToPoseCommand} to drive the
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
            Rotation2d targetHeading = direction.times(-1.0).getAngle();

            return Commands.parallel(
                    led.flash(LEDSignal.AIMING),
                    swerve.driveToPoseCommand(new Pose2d(targetTranslation, targetHeading)));
        });
    }

    /**
     * Creates a command that drives to the far shooting distance from the hub
     * and fires. The robot drives along the line from its current position
     * toward the hub, stopping at the configured distance, then shoots.
     *
     * @param swerve the swerve subsystem
     * @param shooter the shooter subsystem
     * @param ballPath the ball-path subsystem
     * @return a command that drives to shooting position and shoots
     */
    public static Command driveAndShootCommand(LEDSubsystem led,
                                               SwerveSubsystem swerve,
                                               ShooterSubsystem shooter,
                                               BallPathSubsystem ballPath) {

        // phase 1: drive the robot into shooting position and drive the
        // shooter wheel up to speed
        Command phaseOne = Commands.parallel(
                orientToShoot(led, swerve),
                shooter.shootCommand().until(shooter::atSpeed));

        // phase 2: jiggle the robot to agitate balls in the hopper, while
        // keeping the shooter spinning and feeding balls
        Command phaseTwo = Commands.parallel(
//                jiggle(swerve),
                led.flash(LEDSignal.CELEBRATION),
                shooter.shootCommand(),
                ballPath.feedCommand());

        return phaseOne.andThen(phaseTwo);
    }

    /**
     * Jiggle the robot while spinning up the shooter and feeding balls.
     */
    public static Command unload(LEDSubsystem led,
                                 SwerveSubsystem swerve,
                                 ShooterSubsystem shooter,
                                 BallPathSubsystem ballPath) {
        return Commands.parallel(
                shootMode(led, ballPath, shooter),
                jiggle(swerve));
    }

    public static Command orbitCommand(GameController controller, SwerveSubsystem swerve) {
        return orbitCommand(controller, swerve, null);
    }

    /**
     * Creates an orbit drive command that orbits around the hub.
     * <p>
     * This is a compound command that:
     * <ol>
     *   <li>Determines the location of the alliance hub</li>
     *   <li>Rotates to face the hub using {@link SwerveToHeadingCommand}</li>
     *   <li>Orbits the hub using {@link SwerveOrbitCommand}</li>
     * </ol>
     *
     * @param controller the game controller for driver input
     * @return the orbit command sequence
     * @see SwerveOrbitCommand
     */
    public static Command orbitCommand(GameController controller, SwerveSubsystem swerve, Pose2d center) {
        return swerve.defer(() -> {

            // if we already have a center of rotation, use that;
            // otherwise use the center of the hub
            Pose2d hubCenter = center == null
                    ? Field.getHubCenter()
                    : center;

            // calculate heading to face the hub from current position
            Rotation2d headingToHub = hubCenter.getTranslation()
                    .minus(swerve.getPose().getTranslation())
                    .getAngle();

            Util.log("[swerve] orbit: hub at %s, heading %.1f deg",
                    hubCenter, headingToHub.getDegrees());

            // sequence: rotate to face hub, then orbit
            return Commands.sequence(
                    new SwerveToHeadingCommand(swerve, headingToHub),
                    new SwerveOrbitCommand(swerve, controller, center)
            );
        });
    }
}

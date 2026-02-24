package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.Util;

import java.util.function.BooleanSupplier;

import static frc.robot.Config.Launcher.shootDistanceFeet;
import static frc.robot.Config.Launcher.shootDistanceTolerance;
import static frc.robot.Config.Launcher.shootSpinupTime;

/**
 * Factory methods for shooting command sequences.
 */
public class ShootingCommands {

    /**
     * Rapidly oscillates the robot left and right (robot-relative) to
     * dislodge stuck balls. Hold trigger to jiggle, release to stop.
     *
     * @return the jiggle command
     */
    public static Command jiggleCommand(SwerveSubsystem swerve) {

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
     * @return a command that uses a {@link SwerveToHeadingCommand} to turn
     * the robot to face the hub
     */
    public static Command faceHub(SwerveSubsystem swerve) {
        return swerve.defer(() -> {

            Pose2d hubCenter = Field.getHubCenter();
            Pose2d robotPose = swerve.getPose();

            Translation2d hubToRobot = robotPose
                    .getTranslation()
                    .minus(hubCenter.getTranslation());

            return new SwerveToHeadingCommand(swerve, hubToRobot
                    .getAngle()
                    .plus(Rotation2d.k180deg));
        });
    }

    /**
     * @return a command that uses {@link SwerveToPoseCommand} to drive the
     * robot into a shooting position in one fell swoop
     */
    public static Command orientToShoot(SwerveSubsystem swerve) {

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

            return new SwerveToPoseCommand(
                    swerve,
                    new Pose2d(targetTranslation, targetHeading));
        });
    }

    /**
     * Creates a command that drives to the far shooting distance from the hub
     * and fires. The robot drives along the line from its current position
     * toward the hub, stopping at the configured distance, then shoots.
     *
     * @param swerve the swerve subsystem
     * @param launcher the launcher subsystem
     * @return a command that drives to shooting position and shoots
     */
    public static Command driveAndShootCommand(SwerveSubsystem swerve, LauncherSubsystem launcher) {

        // phase 1: drive the robot into shooting position and drive the
        // shooter wheel up to speed
        Command phaseOne = Commands.parallel(
                orientToShoot(swerve),
                launcher
                    .spinUpCommand()
                    .withTimeout(shootSpinupTime.getAsDouble()));

        // phase 2: jiggle the robot to agitate balls in the hopper, while
        // unloading the hopper
        Command phaseTwo = Commands.parallel(
                jiggleCommand(swerve),
                launcher.shootCommand());

        return phaseOne.andThen(phaseTwo);
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
    public Command orbitCommand(GameController controller, SwerveSubsystem swerve) {
        return swerve.defer(() -> {

            // determine the location of the hub
            Pose2d hubCenter = Field.getHubCenter();

            // calculate heading to face the hub from current position
            Rotation2d headingToHub = hubCenter.getTranslation()
                    .minus(swerve.getPose().getTranslation())
                    .getAngle();

            Util.log("[swerve] orbit: hub at %s, heading %.1f deg",
                    hubCenter, headingToHub.getDegrees());

            // sequence: rotate to face hub, then orbit
            return Commands.sequence(
                    new SwerveToHeadingCommand(swerve, headingToHub),
                    new SwerveOrbitCommand(swerve, controller, hubCenter)
            );
        });
    }

    /**
     * @return a command that will turn off the LEDs unless we are within
     * shooting range of the hub
     */
    public static Command flashAtShootingRange(SwerveSubsystem swerve, LEDSubsystem led) {

        BooleanSupplier inRange = () -> {
            double currentDistanceFeet = Util.feetBetween(
                    swerve.getPose(),
                    Field.getHubCenter());
            return MathUtil.isNear(
                    shootDistanceFeet.getAsDouble(),
                    currentDistanceFeet,
                    shootDistanceTolerance.getAsDouble());
        };

        return Commands.either(
                led.show(LEDSignal.SHOOT_RANGE_CLOSE),
                led.show(LEDSignal.OFF),
                inRange);
    }
}

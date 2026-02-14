package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.Util;

import static frc.robot.Config.Shooting.*;

/**
 * Distance-based shooting commands.
 * <p>
 * Calculates distance from the robot to the hub and selects the
 * appropriate RPMs and agitator power for the shot.
 */
public class ShootingCommands {

    /**
     * Represents a type of shot, which corresponds to a specific distance
     * from the center of the hub. Each type has its own lower/upper wheel
     * RPMs and agitator power, all configurable via Preferences.
     */
    public enum ShotType {

        POINT_BLANK,
        CLOSE_RANGE,
        LONG_RANGE;

        /** @return the lower wheel target RPM for this shot type */
        public double lowerRPM() {
            return switch (this) {
                case POINT_BLANK -> pointBlankLowerRPM.getAsDouble();
                case CLOSE_RANGE -> closeLowerRPM.getAsDouble();
                case LONG_RANGE -> farLowerRPM.getAsDouble();
            };
        }

        /** @return the upper wheel target RPM for this shot type */
        public double upperRPM() {
            return switch (this) {
                case POINT_BLANK -> pointBlankUpperRPM.getAsDouble();
                case CLOSE_RANGE -> closeUpperRPM.getAsDouble();
                case LONG_RANGE -> farUpperRPM.getAsDouble();
            };
        }

        /** @return the agitator power percentage for this shot type */
        public double agitatorPower() {
            return switch (this) {
                case POINT_BLANK -> pointBlankAgitator.getAsDouble();
                case CLOSE_RANGE -> closeAgitator.getAsDouble();
                case LONG_RANGE -> farAgitator.getAsDouble();
            };
        }

        /** @return the max distance threshold in feet for this shot type */
        public double distance() {
            return switch (this) {
                case POINT_BLANK -> pointBlankThresholdFeet.getAsDouble();
                case CLOSE_RANGE -> closeThresholdFeet.getAsDouble();
                case LONG_RANGE -> Double.MAX_VALUE;
            };
        }
    }

    /**
     * Selects the appropriate shot type based on distance to the hub.
     *
     * @param distanceFeet the distance to the hub in feet
     * @return the best shot type for that distance
     */
    public static ShotType selectShotType(double distanceFeet) {
        if (distanceFeet <= pointBlankThresholdFeet.getAsDouble()) {
            return ShotType.POINT_BLANK;
        } else if (distanceFeet <= closeThresholdFeet.getAsDouble()) {
            return ShotType.CLOSE_RANGE;
        } else {
            return ShotType.LONG_RANGE;
        }
    }

    /**
     * Spins up the launcher and feeds balls at the specified shot type's
     * RPMs and agitator power. Runs until interrupted.
     *
     * @param launcher the launcher subsystem
     * @param agitator the agitator subsystem
     * @param shotType the type of shot
     * @return a command that spins up and feeds
     */
    public static Command vomitCommand(LauncherSubsystem launcher,
                                       AgitatorSubsystem agitator,
                                       ShotType shotType) {
        return Commands.parallel(
            launcher.spinUpCommand(
                shotType::lowerRPM,
                shotType::upperRPM,
                shotType.name()
            ),
            Commands.sequence(
                Commands.waitUntil(launcher::atSpeed).withTimeout(2.0),
                agitator.feedCommandWithPower(shotType::agitatorPower)
            )
        );
    }

    /**
     * Checks distance to the hub, selects the right shot type, and shoots.
     * Distance is evaluated once when the command starts (not continuously).
     * Runs until interrupted.
     *
     * @param swerve the swerve subsystem (for robot pose)
     * @param launcher the launcher subsystem
     * @param agitator the agitator subsystem
     * @return a distance-aware shoot command
     */
    public static Command makeShotCommand(SwerveSubsystem swerve,
                                          LauncherSubsystem launcher,
                                          AgitatorSubsystem agitator) {
        return Commands.defer(() -> {
            Pose2d robotPose = swerve.getPose();
            Pose2d hubCenter = Field.getHubCenter();
            double distanceFeet = Util.feetBetween(robotPose, hubCenter);
            ShotType selected = selectShotType(distanceFeet);

            Util.log("[shooting] distance=%.1f ft, selected=%s (lower=%.0f, upper=%.0f, agitator=%.0f%%)",
                distanceFeet, selected.name(),
                selected.lowerRPM(), selected.upperRPM(), selected.agitatorPower());

            return vomitCommand(launcher, agitator, selected);
        }, Set.<Subsystem>of(launcher, agitator));
    }
}

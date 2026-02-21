package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;

import static frc.robot.Config.Shooting.*;

/**
 * Factory methods for shooting command sequences.
 */
public class ShootingCommands {

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
        return Commands.defer(() -> {

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
            double farDistMeters = Units.feetToMeters(farDistance.getAsDouble());
            Translation2d targetTranslation = hubCenter.getTranslation()
                    .plus(direction.times(farDistMeters));

            // target heading: face the hub (opposite of direction from hub)
            Rotation2d targetHeading = direction.times(-1.0).getAngle();

            Pose2d targetPose = new Pose2d(targetTranslation, targetHeading);

            return Commands.sequence(
                    swerve.driveToPoseCommand(targetPose),
                    launcher.shootAtRPMCommand(farRPM).withTimeout(3.0)
            );

        }, Set.of(swerve, launcher));
    }
}

package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Util;

import static frc.robot.Config.SwerveTeleop.*;

/**
 * Orbit drive command that orbits around the hub. Right stick controls
 * speed of rotation.
 */
public class SwerveOrbitHubCommand extends Command {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    private final SwerveSubsystem swerve;
    private final GameController controller;
    Pose2d hubCenter;
    double distance;
    double linearSpeed;
    double rotationSpeed;

    /**
     * Creates a {@link SwerveOrbitHubCommand}.
     * @param swerve the swerve subsystem (required)
     * @param controller the game controller for driver input (required)
     */
    public SwerveOrbitHubCommand(SwerveSubsystem swerve, GameController controller) {

        this.swerve = Objects.requireNonNull(swerve);
        this.controller = Objects.requireNonNull(controller);
        this.hubCenter = Util.NAN_POSE;

        addRequirements(swerve);

        if (verboseLogging) {
            SmartDashboard.putData("SwerveOrbitCommand", builder -> {
                builder.addDoubleProperty("Distance", () -> distance, null);
                builder.addDoubleProperty("HubX", () -> Units.metersToInches(hubCenter.getX()), null);
                builder.addDoubleProperty("HubY", () -> Units.metersToInches(hubCenter.getY()), null);
                builder.addDoubleProperty("LinearSpeed", () -> linearSpeed, null);
                builder.addDoubleProperty("RotationSpeed", () -> rotationSpeed, null);
                builder.addBooleanProperty("Running?", this::isScheduled, null);
            });
        }
    }

    @Override
    public void initialize() {
        hubCenter = getHubLocation();
        distance = Util.feetBetween(swerve.getPose(), hubCenter);
    }

    @Override
    public void execute() {

        // get input value
        double input = MathUtil.applyDeadband(
                -controller.getRightX(),
                deadband.getAsDouble());

        // calculate linear speed around the current orbit ring
        linearSpeed = input * maxOrbit.getAsDouble();

        // angular speed depends on distance from the center of rotation
        rotationSpeed = (180.0 / Math.PI) * (linearSpeed / distance);

        // calculate center of rotation relative to robot
        Translation2d center = hubCenter
                .relativeTo(swerve.getPose())
                .getTranslation();

        // you spin me right round, baby right round
        swerve.driveRobotRelative("orbit",
                new ChassisSpeeds(0.0, 0.0, Math.toRadians(rotationSpeed)),
                center);

        Util.publishPose("HubCenter", hubCenter);
    }

    /*
     * Calculates the target point (center of hub position for 2026) for the
     * current alliance
     */
    private Pose2d getHubLocation() {

        Pose2d tag1;
        Pose2d tag2;

        if (Util.isRedAlliance()) {
            tag1 = Util.getTagPose(2);
            tag2 = Util.getTagPose(5);
        } else {
            tag1 = Util.getTagPose(18);
            tag2 = Util.getTagPose(21);
        }

        return new Pose2d(
                (tag1.getX() + tag2.getX()) / 2.0,
                (tag1.getY() + tag2.getY()) / 2.0,
                Rotation2d.kZero);
    }

    @Override
    public void end(boolean interrupted) {
        Util.publishPose("HubCenter", Util.NAN_POSE);
    }
}

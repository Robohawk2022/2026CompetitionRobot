package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
 * Orbit drive command that orbits around a specified center point.
 * <ul>
 *   <li>Right X: controls tangential speed (orbiting around the center)</li>
 *   <li>Right Y: controls radial speed (closer to or further from center)</li>
 * </ul>
 * <p>
 * The center point is specified in field coordinates and is converted to
 * robot-relative coordinates each cycle.
 */
public class SwerveOrbitCommand extends Command {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    private final SwerveSubsystem swerve;
    private final GameController controller;
    private final Pose2d center;

    double distance;
    double tangentialSpeed;
    double radialSpeed;
    double rotationSpeed;

    /**
     * Creates a {@link SwerveOrbitCommand}.
     *
     * @param swerve the swerve subsystem (required)
     * @param controller the game controller for driver input (required)
     * @param center the center of rotation in field coordinates (required)
     */
    public SwerveOrbitCommand(SwerveSubsystem swerve, GameController controller, Pose2d center) {

        this.swerve = Objects.requireNonNull(swerve);
        this.controller = Objects.requireNonNull(controller);
        this.center = Objects.requireNonNull(center);

        addRequirements(swerve);

        if (verboseLogging) {
            SmartDashboard.putData("SwerveOrbitCommand", builder -> {
                builder.addDoubleProperty("Distance", () -> distance, null);
                builder.addDoubleProperty("CenterX", () -> Units.metersToInches(center.getX()), null);
                builder.addDoubleProperty("CenterY", () -> Units.metersToInches(center.getY()), null);
                builder.addDoubleProperty("TangentialSpeed", () -> tangentialSpeed, null);
                builder.addDoubleProperty("RadialSpeed", () -> radialSpeed, null);
                builder.addDoubleProperty("RotationSpeed", () -> rotationSpeed, null);
                builder.addBooleanProperty("Running?", this::isScheduled, null);
            });
        }
    }

    @Override
    public void initialize() {
        distance = Util.feetBetween(swerve.getPose(), center);
        Util.log("[swerve] orbiting center at %s w/ distance %.2f",
                center,
                distance);
    }

    @Override
    public void execute() {

        // calculate center of rotation relative to robot
        Translation2d robotRelativeCenter = center
                .relativeTo(swerve.getPose())
                .getTranslation();

        // update distance (changes as robot moves radially)
        distance = Units.metersToFeet(robotRelativeCenter.getNorm());

        // get joystick inputs with deadband
        double radialInput = MathUtil.applyDeadband(
                -controller.getLeftY(),
                deadband.getAsDouble());
        double tangentialInput = MathUtil.applyDeadband(
                -controller.getRightX(),
                deadband.getAsDouble());

        // calculate radial speed (toward/away from center)
        // positive input (stick forward) moves toward center
        radialSpeed = radialInput * maxOrbit.getAsDouble();

        // calculate tangential speed (orbiting around center)
        tangentialSpeed = tangentialInput * maxOrbit.getAsDouble();

        // angular speed depends on distance from the center of rotation
        rotationSpeed = (180.0 / Math.PI) * (tangentialSpeed / distance);

        // calculate radial velocity components (toward center)
        // normalize the center vector to get direction, then scale by speed
        double radialVelocityMps = Units.feetToMeters(radialSpeed);
        Translation2d radialDirection = robotRelativeCenter.div(robotRelativeCenter.getNorm());
        double vx = radialDirection.getX() * radialVelocityMps;
        double vy = radialDirection.getY() * radialVelocityMps;

        // combine radial translation with rotation around center
        swerve.driveRobotRelative("orbit",
                new ChassisSpeeds(vx, vy, Math.toRadians(rotationSpeed)),
                robotRelativeCenter);

        Util.publishPose("OrbitCenter", center);
    }

    @Override
    public void end(boolean interrupted) {
        Util.publishPose("OrbitCenter", Util.NAN_POSE);
    }
}

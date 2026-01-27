package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Util;

import static frc.robot.Config.Swerve.*;

/**
 * Teleop drive command for swerve drive with turbo/sniper speed modes.
 * <p>
 * Applies deadband, axis inversion, and speed modifiers based on trigger inputs:
 * <ul>
 *   <li>Left trigger: Sniper mode (slow, precise movement)</li>
 *   <li>Right trigger: Turbo mode (fast movement)</li>
 *   <li>Sniper takes priority if both triggers are pressed</li>
 * </ul>
 */
public class SwerveTeleopCommand extends Command {

    private final SwerveSubsystem swerve;
    private final GameController controller;

    /**
     * Creates a {@link SwerveTeleopCommand}.
     *
     * @param swerve     the swerve subsystem (required)
     * @param controller the game controller for driver input (required)
     */
    public SwerveTeleopCommand(SwerveSubsystem swerve, GameController controller) {
        this.swerve = Objects.requireNonNull(swerve);
        this.controller = Objects.requireNonNull(controller);
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // read speed limits from config
        double maxSpeedMps = Units.feetToMeters(maxSpeedFps.getAsDouble());
        double maxRotationRps = Math.toRadians(maxRotationDps.getAsDouble());

        // get controller inputs with deadband and inversion
        double x = -MathUtil.applyDeadband(controller.getLeftY(), deadzone.getAsDouble());
        double y = -MathUtil.applyDeadband(controller.getLeftX(), deadzone.getAsDouble());
        double rot = -MathUtil.applyDeadband(controller.getRightX(), deadzone.getAsDouble());

        // check trigger thresholds
        boolean sniperActive = controller.leftTriggerSupplier().getAsBoolean();
        boolean turboActive = controller.rightTriggerSupplier().getAsBoolean();

        // clamp inputs to [-1, 1]
        x = MathUtil.clamp(x, -1.0, 1.0);
        y = MathUtil.clamp(y, -1.0, 1.0);
        rot = MathUtil.clamp(rot, -1.0, 1.0);

        // convert to velocities
        double vx = x * maxSpeedMps;
        double vy = y * maxSpeedMps;
        double omega = rot * maxRotationRps;

        // track effective max speed for desaturation
        double effectiveMaxSpeed = maxSpeedMps;

        // apply speed modifiers (sniper takes priority)
        String mode;
        if (sniperActive) {
            double sf = sniperFactor.getAsDouble();
            vx *= sf;
            vy *= sf;
            effectiveMaxSpeed *= sf;
            if (applySniperToRotation.getAsBoolean()) {
                omega *= sf;
            }
            mode = "teleop-sniper";
        } else if (turboActive) {
            double tf = turboFactor.getAsDouble();
            vx *= tf;
            vy *= tf;
            effectiveMaxSpeed *= tf;
            mode = "teleop-turbo";
        } else {
            mode = "teleop-field";
        }

        // always do field relative
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
        speeds = Util.fromDriverRelativeSpeeds(speeds, swerve.getHeading());
        swerve.driveRobotRelative("teleop", speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

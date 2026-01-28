package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.TeleopInput;
import frc.robot.util.TeleopInput.TurboSniperMode;
import frc.robot.util.Util;

import static frc.robot.Config.SwerveTeleop.*;

/**
 * Teleop drive command for swerve drive that provides:
 * <ul>
 *
 *     <li>Max translate/rotate speed (to convert 0.0 - 1.0 values to
 *     a valid field speed)</li>
 *
 *     <li>Deadband (highly recommended for joysticks that don't
 *     report 0.0 values when centered)</li>
 *
 *     <li>Exponent (lore has it that squaring or cubing small
 *     input values gives better control)</li>
 *
 *     <li>"Turbo" and "Sniper" Factors for translation (allows you to
 *     implement stuff like a simple "double my top speed" control)</li>
 *
 * </ul>
 *
 * Uses the traditional time-honored controller mapping of our people:
 * <ul>
 *     <li>Left stick controls translation</li>
 *     <li>Right stick controls rotation</li>
 *     <li>Left trigger enables sniper mode</li>
 *     <li>Right trigger enables turbo mode</li>
 * </ul>
 */
public class SwerveTeleopCommand extends Command {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    final SwerveSubsystem swerve;
    final GameController controller;
    double inputX;
    double inputY;
    double inputOmega;
    double speedX;
    double speedY;
    double speedOmega;

    /**
     * Creates a {@link SwerveTeleopCommand}.
     * @param swerve the swerve subsystem (required)
     * @param controller the game controller for driver input (required)
     */
    public SwerveTeleopCommand(SwerveSubsystem swerve, GameController controller) {

        this.swerve = Objects.requireNonNull(swerve);
        this.controller = Objects.requireNonNull(controller);

        addRequirements(swerve);

        SmartDashboard.putData("SwerveTeleopCommand", builder -> {
            builder.addBooleanProperty("Running?", this::isScheduled, null);
            builder.addStringProperty("Mode", () -> Objects.toString(TeleopInput.getMode(controller)), null);
            if (verboseLogging) {
                builder.addDoubleProperty("InputOmega", () -> inputOmega, null);
                builder.addDoubleProperty("InputX", () -> inputX, null);
                builder.addDoubleProperty("InputY", () -> inputY, null);
                builder.addDoubleProperty("SpeedOmega", () -> speedOmega, null);
                builder.addDoubleProperty("SpeedX", () -> speedX, null);
                builder.addDoubleProperty("SpeedY", () -> speedY, null);
            }
        });
    }

    @Override
    public void initialize() {
        Util.log("[swerve] entering teleop");
    }

    @Override
    public void execute() {

        // get conditioned joystick input
        inputX = TeleopInput.conditionInput(-controller.getLeftY());
        inputY = TeleopInput.conditionInput(-controller.getLeftX());
        inputOmega = TeleopInput.conditionInput(-controller.getRightX());

        // ensure that the point defined by (x, y) lies on the unit
        // circle - when we scale them by the maximum translate speed
        // this will prevent us from shooting off too fast at an angle
        double d = Math.hypot(inputX, inputY);
        if (d > 1.0) {
            inputX /= d;
            inputY /= d;
        }

        // calculate speeds with mode factor applied
        TurboSniperMode mode = TeleopInput.getMode(controller);
        double sf = mode.getSpeedFactor();

        speedX = inputX * maxTranslate.getAsDouble() * sf;
        speedY = inputY * maxTranslate.getAsDouble() * sf;
        if (mode.applyFactorToRotation()) {
            speedOmega = inputOmega * maxRotate.getAsDouble() * sf;
        }

        ChassisSpeeds speeds = new ChassisSpeeds(
                Units.feetToMeters(speedX),
                Units.feetToMeters(speedY),
                Math.toRadians(speedOmega));

        // convert from driver relative speeds if required
        if (driverRelative.getAsBoolean()) {
            speeds = Util.fromDriverRelativeSpeeds(speeds, swerve.getHeading());
        }

        swerve.driveRobotRelative("teleop", speeds);
    }

}

package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
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

    /** Mode */
    enum Mode {
        SNIPER,
        TURBO,
        NORMAL
    }

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
            builder.addStringProperty("Mode", () -> Objects.toString(getMode()), null);
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
    public void execute() {

        // get conditions joystick input
        inputX = conditionInput(-controller.getLeftY());
        inputY = conditionInput(-controller.getLeftX());
        inputOmega = conditionInput(-controller.getRightX());

        // ensure that the point defined by (x, y) lies on the unit
        // circle - when we scale them by the maximum translate speed
        // this will prevent us from shooting off too fast at an angle
        double d = Math.hypot(inputX, inputY);
        if (d > 1.0) {
            inputX /= d;
            inputY /= d;
        }

        // calculate speeds
        double mt = maxTranslate.getAsDouble();
        speedX = inputX * mt;
        speedY = inputY * mt;
        speedOmega = inputOmega * maxRotate.getAsDouble();

        // increase/decrease for turbo/sniper mode
        switch (getMode()) {

            // if we're in sniper mode, we might want to slow down rotation too
            case SNIPER:
                double sf = sniperFactor.getAsDouble();
                speedX *= sf;
                speedY *= sf;
                if (applySniperToRotation.getAsBoolean()) {
                    speedOmega *= sf;
                }
                break;

            // if we're in turbo mode, we only change translation
            case TURBO:
                double tf = turboFactor.getAsDouble();
                speedX *= tf;
                speedY *= tf;
                break;

            // nothing to do here
            case NORMAL:
                break;

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

    /*
     * Gets the operating mode
     */
    private Mode getMode() {
        if (controller.leftTriggerSupplier().getAsBoolean()) {
            return Mode.SNIPER;
        }
        if (controller.rightTriggerSupplier().getAsBoolean()) {
            return Mode.TURBO;
        }
        return Mode.NORMAL;
    }

    /*
     * Applies deadband and exponent to input
     */
    private double conditionInput(double input) {
        double d1 = MathUtil.clamp(input, -1.0, 1.0);
        double d2 = MathUtil.applyDeadband(d1, deadband.getAsDouble());
        double d3 = Math.copySign(Math.pow(d2, exponent.getAsDouble()), input);
        return d3;
    }
}

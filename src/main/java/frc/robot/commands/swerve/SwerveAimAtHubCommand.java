package frc.robot.commands.swerve;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameController;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.PDController;
import frc.robot.util.TeleopInput;
import frc.robot.util.TeleopInput.TurboSniperMode;
import frc.robot.util.Util;

import static frc.robot.Config.Shooting.*;
import static frc.robot.Config.SwerveAim.*;
import static frc.robot.Config.SwerveTeleop.*;

/**
 * Teleop drive command that auto-aims the robot to face the hub center.
 * <ul>
 *   <li>Left stick: translation (same as normal teleop)</li>
 *   <li>Rotation: automatically controlled to face the hub</li>
 *   <li>Left trigger: sniper mode, Right trigger: turbo mode</li>
 * </ul>
 *
 * <p>Uses the robot's odometry pose and {@link Field#getHubCenter()} to
 * continuously compute the desired heading, then a {@link PDController}
 * to drive rotation toward that heading.</p>
 *
 * <p>When an {@link LEDSubsystem} is provided, the LEDs turn green when
 * the robot is within tolerance of a shooting distance, and red otherwise.</p>
 */
public class SwerveAimAtHubCommand extends Command {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    final SwerveSubsystem swerve;
    final LEDSubsystem led;
    final GameController controller;
    final PDController headingPid;

    double inputX;
    double inputY;
    double speedX;
    double speedY;
    double headingCurrentDeg;
    double headingGoalDeg;
    double headingErrorDeg;
    double omegaDps;
    double distanceToHub;
    String shotZone;

    /**
     * Creates a {@link SwerveAimAtHubCommand}.
     *
     * @param swerve the swerve subsystem (required)
     * @param led the LED subsystem for distance feedback (nullable)
     * @param controller the game controller for driver input (required)
     */
    public SwerveAimAtHubCommand(SwerveSubsystem swerve, LEDSubsystem led, GameController controller) {

        this.swerve = Objects.requireNonNull(swerve);
        this.led = led;
        this.controller = Objects.requireNonNull(controller);
        this.headingPid = new PDController(kP, kD, headingTolerance);
        this.shotZone = "none";

        addRequirements(swerve);

        SmartDashboard.putData("SwerveAimAtHubCommand", builder -> {
            builder.addBooleanProperty("Running?", this::isScheduled, null);
            builder.addStringProperty("Mode", () -> Objects.toString(TeleopInput.getMode(controller)), null);
            builder.addDoubleProperty("DistanceToHub", () -> distanceToHub, null);
            builder.addStringProperty("ShotZone", () -> shotZone, null);
            builder.addDoubleProperty("HeadingCurrent", () -> headingCurrentDeg, null);
            builder.addDoubleProperty("HeadingGoal", () -> headingGoalDeg, null);
            builder.addDoubleProperty("HeadingError", () -> headingErrorDeg, null);
            if (verboseLogging) {
                builder.addDoubleProperty("OmegaDps", () -> omegaDps, null);
                builder.addDoubleProperty("SpeedX", () -> speedX, null);
                builder.addDoubleProperty("SpeedY", () -> speedY, null);
            }
        });
    }

    @Override
    public void initialize() {
        headingPid.reset();
        shotZone = "none";
        Util.log("[swerve] aim at hub: started");
    }

    @Override
    public void execute() {

        // compute heading to hub from current pose
        Pose2d hubCenter = Field.getHubCenter();
        Pose2d robotPose = swerve.getPose();

        Rotation2d headingToHub = hubCenter.getTranslation()
                .minus(robotPose.getTranslation())
                .getAngle();

        headingGoalDeg = headingToHub.getDegrees();
        headingCurrentDeg = swerve.getHeading().getDegrees();
        headingErrorDeg = Util.degreeModulus(headingGoalDeg - headingCurrentDeg);
        distanceToHub = Util.feetBetween(robotPose, hubCenter);

        // update LED based on distance to hub
        updateShotZone();

        // PD feedback on heading error
        omegaDps = headingPid.calculate(0.0, -headingErrorDeg);

        // clamp rotation speed
        double maxOmega = maxRotateDps.getAsDouble();
        omegaDps = Math.max(-maxOmega, Math.min(maxOmega, omegaDps));

        // get conditioned translation input from driver
        inputX = TeleopInput.conditionInput(-controller.getLeftY());
        inputY = TeleopInput.conditionInput(-controller.getLeftX());

        // clamp to unit circle
        double d = Math.hypot(inputX, inputY);
        if (d > 1.0) {
            inputX /= d;
            inputY /= d;
        }

        // apply speed factors
        TurboSniperMode mode = TeleopInput.getMode(controller);
        double sf = mode.getSpeedFactor();
        speedX = inputX * maxTranslate.getAsDouble() * sf;
        speedY = inputY * maxTranslate.getAsDouble() * sf;

        ChassisSpeeds speeds = new ChassisSpeeds(
                Units.feetToMeters(speedX),
                Units.feetToMeters(speedY),
                Math.toRadians(omegaDps));

        if (driverRelative.getAsBoolean()) {
            swerve.driveFieldRelative("aim-hub", speeds);
        } else {
            swerve.driveRobotRelative("aim-hub", speeds);
        }

        Util.publishPose("AimTarget", hubCenter);
    }

    /**
     * Checks distance to hub and updates LED signal accordingly.
     * Green when within tolerance of a shooting distance, red otherwise.
     */
    private void updateShotZone() {
        if (led == null) return;

        double tol = distanceTolerance.getAsDouble();
        double closeDist = closeDistance.getAsDouble();
        double farDist = farDistance.getAsDouble();

        if (Math.abs(distanceToHub - closeDist) <= tol) {
            shotZone = "close";
            led.setSignal(LEDSignal.SHOOT_RANGE_CLOSE);
        } else if (Math.abs(distanceToHub - farDist) <= tol) {
            shotZone = "far";
            led.setSignal(LEDSignal.SHOOT_RANGE_CLOSE);
        } else {
            shotZone = "none";
            led.setSignal(LEDSignal.SHOOT_RANGE_FAR);
        }
    }

    /**
     * @return the recommended shooter RPM based on current distance zone,
     *         or 0 if not in a shooting zone
     */
    public double getRecommendedRPM() {
        return switch (shotZone) {
            case "close" -> closeRPM.getAsDouble();
            case "far" -> farRPM.getAsDouble();
            default -> 0;
        };
    }

    /**
     * @return true if the robot is in a valid shooting zone
     */
    public boolean inShootingZone() {
        return !shotZone.equals("none");
    }

    @Override
    public void end(boolean interrupted) {
        Util.publishPose("AimTarget", Util.NAN_POSE);
        if (led != null) {
            led.setSignal(LEDSignal.OFF);
        }
        Util.log("[swerve] aim at hub: %s", interrupted ? "interrupted" : "ended");
    }
}

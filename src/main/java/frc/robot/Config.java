package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import static frc.robot.util.Util.pref;

public interface Config {

//region Logging ---------------------------------------------------------------

    interface Logging {

        /** Master switch for all verbose logging (overrides individual settings when false) */
        BooleanSupplier verboseEnabled = pref("Logging/VerboseEnabled?", true);

        /** Log controller button presses and stick values */
        BooleanSupplier controllerLogging = pref("Logging/Controller?", true);

        /** Log command scheduling and lifecycle events */
        BooleanSupplier commandLogging = pref("Logging/Commands?", true);

        /** Log every command execute call (very verbose, use sparingly) */
        BooleanSupplier commandExecuteLogging = pref("Logging/CommandExecute?", false);

        /**
         * Check if a specific logging category is enabled.
         * Respects the master switch.
         */
        static boolean isEnabled(BooleanSupplier category) {
            return verboseEnabled.getAsBoolean() && category.getAsBoolean();
        }
    }

//endregion

//region Swerve (General) ------------------------------------------------------

    interface Swerve {

        /** Enable cosine compensation - scales drive output by cos(angle error) */
        BooleanSupplier cosineCompensation = pref("Swerve/CosineCompensation?", true);

        /** Minimum speed (m/s) to command the angle motor - prevents jitter when stationary */
        DoubleSupplier minSpeedForAngle = pref("Swerve/MinSpeedForAngle", 0.01);

        /** Maximum pose jump (vision corrections larger than this are rejected) */
        DoubleSupplier maxPoseJumpFeet = pref("Limelight/MaxPoseJumpFeet", 2.0);

    }

//endregion

//region Swerve (Teleop) -------------------------------------------------------

    interface SwerveTeleop {

        /** Maximum speeds */
        DoubleSupplier maxTranslate = pref("SwerveTeleop/MaxTranslateFPS", 10.0);
        DoubleSupplier maxRotate = pref("SwerveTeleop/MaxRotateDPS", 180.0);
        DoubleSupplier maxOrbit = pref("SwerveTeleop/MaxOrbitFPS", 15.0);

        /** Speed mode factors (turbo/sniper) */
        DoubleSupplier sniperFactor = pref("SwerveTeleop/SniperFactor", 0.25);
        DoubleSupplier turboFactor = pref("SwerveTeleop/TurboFactor", 1.5);
        BooleanSupplier applySniperToRotation = pref("SwerveTeleop/SniperRotation?", true);

        /** Driver relative mode */
        BooleanSupplier driverRelative = pref("SwerveTeleop/DriverRelative?", true);

        /** Joystick deadband & exponent */
        DoubleSupplier deadband = pref("SwerveTeleop/Deadband", 0.1);
        DoubleSupplier exponent = pref("SwerveTeleop/Exponent", 2.0);

        /** Use XBox mapping */
        BooleanSupplier useXboxMapping = pref("SwerveTeleop/UseXboxMapping?", true);

    }

//endregion

//region Swerve (Auto) ---------------------------------------------------------

    interface SwerveAuto {

        /** Maximum translation velocity in feet per second */
        DoubleSupplier maxTranslationVelocity = pref("SwerveAuto/MaxTranslationFPS", 2.0);

        /** Maximum translation acceleration in feet per second squared */
        DoubleSupplier maxTranslationAcceleration = pref("SwerveAuto/MaxTranslationFPSS", 4.0);

        /** Proportional gain for translation feedback (units: 1/sec) */
        DoubleSupplier translationKp = pref("SwerveAuto/TranslationKp", 0.0);

        /** Tolerance for position commands in feet */
        DoubleSupplier positionTolerance = pref("SwerveAuto/PositionToleranceFeet", 0.1);

        /** Maximum rotation velocity in degrees per second */
        DoubleSupplier maxRotationVelocity = pref("SwerveAuto/MaxRotationDPS", 90.0);

        /** Maximum rotation acceleration in degrees per second squared */
        DoubleSupplier maxRotationAcceleration = pref("SwerveAuto/MaxRotationDPSS", 180.0);

        /** Proportional gain for rotation feedback (units: 1/sec) */
        DoubleSupplier rotationKp = pref("SwerveAuto/RotationKp", 0.0);

        /** Tolerance for heading commands in degrees */
        DoubleSupplier headingTolerance = pref("SwerveAuto/HeadingToleranceDeg", 2.0);

    }

//endregion

//region Swerve (PathPlanner) --------------------------------------------------

    interface PathPlanner {

        /** Maximum speed in meters/sec */
        DoubleSupplier maxSpeed = pref("PathPlanner/MaxSpeed", 3.5);

        /** Translation gains for path following */
        DoubleSupplier translationP = pref("PathPlanner/Translation/kP", 0.0);
        DoubleSupplier translationI = pref("PathPlanner/Translation/kI", 0.0);
        DoubleSupplier translationD = pref("PathPlanner/Translation/kD", 0.0);

        /** Rotation gains for path following */
        DoubleSupplier rotationP = pref("PathPlanner/Rotation/kP", 0.0);
        DoubleSupplier rotationI = pref("PathPlanner/Rotation/kI", 0.0);
        DoubleSupplier rotationD = pref("PathPlanner/Rotation/kD", 0.0);

        /** Enable PathPlanner debug logging */
        BooleanSupplier debugLogging = pref("PathPlanner/Debug?", true);

    }

//endregion

//region Limelight & Sim -------------------------------------------------------

    interface Limelight {

        /** Name of the Limelight in NetworkTables */
        String limelightName = "limelight";

        /** Master enable/disable for Limelight */
        BooleanSupplier enabled = pref("Limelight/Enabled?", true);

        /** "Close" range for pose confidence (in feet) */
        DoubleSupplier closeRange = pref("Limelight/CloseRange", 10.0);

        /** Maximum yaw rate for MegaTag2 estimates (degrees per second) */
        DoubleSupplier maxYawRate = pref("Limelight/MaxYawRate", 720.0);

        /** Confidence levels (lower numbers mean higher confidence) */
        // X, Y in meters; Theta in radians (0.5 rad ≈ 28°, 0.3 rad ≈ 17°, 0.1 rad ≈ 6°)
        Vector<N3> lowConfidence = VecBuilder.fill(2.0, 2.0, 0.5);
        Vector<N3> mediumConfidence = VecBuilder.fill(0.9, 0.9, 0.3);
        Vector<N3> highConfidence = VecBuilder.fill(0.5, 0.5, 0.1);

    }

    interface LimelightSim {

        /** Offsets from robot center to camera (meters) */
        double cameraForwardOffset = Units.inchesToMeters(4.0);
        double cameraSideOffset = Units.inchesToMeters(0.0);
        double cameraHeight = Units.inchesToMeters(24.0);
        double cameraPitch = Math.toRadians(20.0);

        /** Horizontal/vertical field of view (degrees) */
        double horizontalFov = 63.3;
        double verticalFov = 50.0;

        /** Min/max distance to detect tags (meters) */
        DoubleSupplier maxDetectionDistance = pref("LimelightSim/MaxDistance", 5.0);
        DoubleSupplier minDetectionDistance = pref("LimelightSim/MinDistance", 0.3);

        /** Maximum angle between camera and tag normal for detection (degrees) */
        DoubleSupplier maxTagAngle = pref("LimelightSim/MaxTagAngle", 60.0);

        /** Standard deviation for position noise (meters) */
        DoubleSupplier positionNoiseStdDev = pref("LimelightSim/PositionNoise", 0.02);

        /** Standard deviation for rotation noise (degrees) */
        DoubleSupplier rotationNoiseStdDev = pref("LimelightSim/RotationNoise", 1.0);

        /** Simulated pipeline latency (milliseconds) */
        DoubleSupplier pipelineLatency = pref("LimelightSim/LatencyMs", 25.0);

        /** Probability of dropping a frame (0.0-1.0) */
        DoubleSupplier frameDropProbability = pref("LimelightSim/FrameDropProb", 0.05);

        /** Enable/disable the Limelight simulation */
        BooleanSupplier enabled = pref("LimelightSim/Enabled?", true);

        /** AprilTag size */
        double tagSize = Units.inchesToMeters(6.5);

    }

    /**
     * Configuration for the QuestNav and pose estimation
     */
    interface QuestNav {

        /** Master enable/disable for QuestNav */
        BooleanSupplier enabled = pref("QuestNav/Enabled?", false);

        /** Transformation based on mounting of camera on robot */
        Transform3d robotToQuestTransform = new Transform3d();
        Transform3d questToRobotTransform = robotToQuestTransform.inverse();

        /** Confidence for position estimates */
        Vector<N3> confidence = VecBuilder.fill(0.02, 0.02, 0.035);

    }

//endregion

//region LED -------------------------------------------------------------------

    interface LED {

        /** PWM port for the REV Blinkin LED controller */
        int pwmPort = 0;

    }

//endregion

//region Intake ----------------------------------------------------------------

    interface IntakeFront {

        /** CAN ID for the intake motor */
        int MOTOR_CAN_ID = 20;

        //=======================================================================
        // Speed settings (in revolutions per second)
        //=======================================================================

        /** Target speed for intake (sucking in) in revolutions per second */
        DoubleSupplier intakeSpeedRPS = pref("IntakeFront/IntakeSpeedRPS", 80.0);

        /** Target speed for eject (spitting out) in revolutions per second */
        DoubleSupplier ejectSpeedRPS = pref("IntakeFront/EjectSpeedRPS", 60.0);

        /**
         * Invert the motor direction.
         * <p>
         * Toggle this if the motor spins the wrong way (ejecting instead of intaking).
         */
        BooleanSupplier inverted = pref("IntakeFront/Inverted?", false);

        //=======================================================================
        // Closed-loop PID gains
        //=======================================================================

        /** Velocity feedforward gain (volts per rev/sec) */
        DoubleSupplier kV = pref("IntakeFront/kV", 0.12);

        /** Proportional gain for velocity control */
        DoubleSupplier kP = pref("IntakeFront/kP", 0.1);

        //=======================================================================
        // Stall detection
        //=======================================================================

        /** Velocity threshold below which motor is considered stalled (RPM) */
        DoubleSupplier stallThresholdRPM = pref("IntakeFront/StallThresholdRPM", 100.0);

        /** Time motor must be stalled before triggering stall state (seconds) */
        DoubleSupplier stallTimeSec = pref("IntakeFront/StallTimeSec", 0.25);

        //=======================================================================
        // Motor limits
        //=======================================================================

        /** Current limit for the intake motor in amps */
        DoubleSupplier currentLimit = pref("IntakeFront/CurrentLimit", 40.0);

    }

//endregion

//region HubbardShooter --------------------------------------------------------

    /**
     * Configuration for the HubbardShooter subsystem.
     * <p>
     * Two NEO motors (CAN 51, 52) with 12:1 gear reduction (3:1 * 4:1).
     * Open-loop voltage control for intake, outtake, and shooting modes.
     */
    interface HubbardShooter {

        /** CAN IDs for the shooter motors */
        int MOTOR_1_CAN_ID = 51;
        int MOTOR_2_CAN_ID = 52;

        //=======================================================================
        // Power settings (percentage 0-100)
        //=======================================================================

        /** Power for intake mode (both motors same direction) */
        DoubleSupplier intakePower = pref("HubbardShooter/IntakePower%", 20.0);

        /** Power for outtake mode (both motors same direction, reversed) */
        DoubleSupplier outtakePower = pref("HubbardShooter/OuttakePower%", 20.0);

        /** Power for shooting mode (motors counter-rotate) */
        DoubleSupplier shootingPower = pref("HubbardShooter/ShootingPower%", 50.0);

        //=======================================================================
        // Motor configuration
        //=======================================================================

        /** Invert motor 1 direction */
        BooleanSupplier motor1Inverted = pref("HubbardShooter/Motor1Inverted?", false);

        /** Invert motor 2 direction */
        BooleanSupplier motor2Inverted = pref("HubbardShooter/Motor2Inverted?", false);

        /** Current limit in amps */
        DoubleSupplier currentLimit = pref("HubbardShooter/CurrentLimit", 40.0);

    }

//endregion

}

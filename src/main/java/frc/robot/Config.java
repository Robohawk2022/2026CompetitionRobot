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

//region Shooter ---------------------------------------------------------------

    interface Shooter {

        /** Physical properties of the mechanism */
        double gearRatio = 1.0 / 3.0;
        double wheelDiameter = 4.0 / 12.0;
        double wheelCircumference = wheelDiameter * Math.PI;

        /** Feedforward/feedback constants for speed */
        DoubleSupplier p = pref("ShooterSubsystem/kP", 0.0);
        DoubleSupplier d = pref("ShooterSubsystem/kD", 0.0);
        DoubleSupplier v = pref("ShooterSubsystem/kV", 0.0);
        DoubleSupplier tolerance = pref("ShooterSubsystem/Tolerance", 0.0);

        /** Preset speeds in feet per seconds */
        DoubleSupplier speed1 = pref("ShooterPresets/Speed1", 10.0);
        DoubleSupplier speed2 = pref("ShooterPresets/Speed1", 20.0);
        DoubleSupplier speed3 = pref("ShooterPresets/Speed3", 3.0);

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

//region HubberdShooter ---------------------------------------------------------

    /**
     * Configuration for the HubberdShooter subsystem.
     * <p>
     * Two Falcon 500 motors that can operate in four modes:
     * <ul>
     *   <li>Off - Motors stopped</li>
     *   <li>Intake - Both motors same direction at 20% power</li>
     *   <li>Outtake - Both motors opposite direction at 20% power</li>
     *   <li>Shooting - Motors counter-rotate at 600 RPM</li>
     * </ul>
     */
    interface HubberdShooter {

        /** CAN IDs for the shooter motors */
        int MOTOR_1_CAN_ID = 51;
        int MOTOR_2_CAN_ID = 52;

        //=======================================================================
        // Power settings (percentage 0-100)
        //=======================================================================

        /** Power for intake mode (both motors same direction) */
        DoubleSupplier intakePower = pref("HubberdShooter/IntakePower%", 20.0);

        /** Power for outtake mode (both motors same direction, reversed) */
        DoubleSupplier outtakePower = pref("HubberdShooter/OuttakePower%", 20.0);

        /** Power for shooting mode (motors counter-rotate) */
        DoubleSupplier shootingPower = pref("HubberdShooter/ShootingPower%", 50.0);

        //=======================================================================
        // Motor configuration
        //=======================================================================

        /** Invert motor 1 direction */
        BooleanSupplier motor1Inverted = pref("HubberdShooter/Motor1Inverted?", false);

        /** Invert motor 2 direction */
        BooleanSupplier motor2Inverted = pref("HubberdShooter/Motor2Inverted?", false);

        //=======================================================================
        // Current limits
        //=======================================================================

        /** Stator current limit in amps */
        DoubleSupplier statorCurrentLimit = pref("HubberdShooter/StatorCurrentLimit", 60.0);

        /** Supply current limit in amps */
        DoubleSupplier supplyCurrentLimit = pref("HubberdShooter/SupplyCurrentLimit", 40.0);

    }

//endregion

//region Launcher --------------------------------------------------------------

    /**
     * Configuration for the Launcher subsystem.
     * <p>
     * Three NEO motors: agitator, lower wheel, upper wheel.
     * The lower wheel doubles as intake by spinning backward.
     * Different upper/lower wheel speeds control ball arc via Magnus effect.
     * Wheels use closed-loop velocity control (feedforward + PD feedback).
     * Agitator uses open-loop voltage control.
     */
    interface Launcher {

        // TODO update CAN IDs once the build team wires the motors
        int AGITATOR_CAN_ID = 31;
        int LOWER_WHEEL_CAN_ID = 32;
        int UPPER_WHEEL_CAN_ID = 33;

        //=======================================================================
        // Intake & agitator
        //=======================================================================

        /** RPM for lower wheel when intaking (spins backward to pull balls in) */
        DoubleSupplier intakeSpeedRPM = pref("Launcher/IntakeSpeedRPM", 1500.0);

        /** Power for agitator motor when feeding balls toward wheels */
        DoubleSupplier agitatorPower = pref("Launcher/AgitatorPower%", 30.0);

        /** Power for agitator when actively feeding a shot */
        DoubleSupplier feedPower = pref("Launcher/FeedPower%", 50.0);

        //=======================================================================
        // Wheel velocity PID - closed loop
        //=======================================================================

        /** Feedforward: volts per RPM (V / RPM). Start with 12.0 / 5676 = ~0.002 for NEO */
        DoubleSupplier kV = pref("Launcher/Wheels/kV", 0.002);

        /** Proportional feedback: volts per RPM of error */
        DoubleSupplier kP = pref("Launcher/Wheels/kP", 0.001);

        /** Derivative feedback: volts per RPM/s of error change rate */
        DoubleSupplier kD = pref("Launcher/Wheels/kD", 0.0);

        /** RPM tolerance for "at speed" check */
        DoubleSupplier tolerance = pref("Launcher/Wheels/ToleranceRPM", 100.0);

        //=======================================================================
        // Shot presets - target RPMs for lower/upper wheels
        //=======================================================================

        /** High arc: upper faster = backspin = more lift */
        DoubleSupplier highArcLowerRPM = pref("Launcher/HighArc/LowerRPM", 2000.0);
        DoubleSupplier highArcUpperRPM = pref("Launcher/HighArc/UpperRPM", 3500.0);

        /** Flat shot: lower faster = topspin = flatter trajectory */
        DoubleSupplier flatLowerRPM = pref("Launcher/Flat/LowerRPM", 3500.0);
        DoubleSupplier flatUpperRPM = pref("Launcher/Flat/UpperRPM", 2000.0);

        /** Neutral: equal speed = no spin bias */
        DoubleSupplier neutralRPM = pref("Launcher/Neutral/RPM", 3000.0);

        //=======================================================================
        // Motor configuration
        //=======================================================================

        BooleanSupplier agitatorInverted = pref("Launcher/AgitatorInverted?", false);
        BooleanSupplier lowerWheelInverted = pref("Launcher/LowerWheelInverted?", false);
        BooleanSupplier upperWheelInverted = pref("Launcher/UpperWheelInverted?", false);

        /** Current limit for all launcher motors in amps */
        DoubleSupplier currentLimit = pref("Launcher/CurrentLimit", 40.0);

    }

//endregion

}

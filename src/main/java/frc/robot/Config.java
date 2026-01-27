package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import static frc.robot.util.Util.pref;

public interface Config {

    //===========================================================================
    // GLOBAL LOGGING SETTINGS (at top for easy access)
    //===========================================================================

    /**
     * Global logging configuration.
     * <p>
     * Enable/disable verbose logging for various subsystems from one place.
     */
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

//region Swerve ----------------------------------------------------------------

    interface SwerveTeleop {

        /** Maximum speeds */
        DoubleSupplier maxTranslate = pref("SwerveTeleop/MaxTranslateFPS", 10.0);
        DoubleSupplier maxRotate = pref("SwerveTeleop/MaxRotateDPS", 180.0);
        DoubleSupplier maxOrbit = pref("SwerveTeleop/MaxOrbitFPS", 8.0);

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

    /**
     * Configuration for the swerve drive subsystem.
     * <p>
     * Hardware-specific constants (CAN IDs, offsets, PID gains) are in
     * {@link frc.robot.subsystems.swerve.SwerveHardwareConfig}.
     */
    interface Swerve {

        //=======================================================================
        // Tunable values (adjustable via dashboard/preferences)
        //=======================================================================

        DoubleSupplier maxSpeedFps = pref("Swerve/MaxSpeedFPS", 15.0);
        DoubleSupplier maxRotationDps = pref("Swerve/MaxRotationDPS", 360.0);

        // Module optimization settings
        /** Enable cosine compensation - scales drive output by cos(angle error) */
        BooleanSupplier cosineCompensation = pref("Swerve/CosineCompensation?", true);
        /** Minimum speed (m/s) to command the angle motor - prevents jitter when stationary */
        DoubleSupplier minSpeedForAngle = pref("Swerve/MinSpeedForAngle", 0.01);

        //=======================================================================
        // Physical constants (change only if hardware changes)
        //=======================================================================

        // Chassis dimensions (inches) - measure from wheel center to wheel center
        double WHEEL_BASE_INCHES = 19.0 + 7.0 / 16.0;       // Front-to-back distance
        double TRACK_WIDTH_INCHES = 25.0;       // Left-to-right distance

        double WHEEL_DIAMETER_METERS = 0.1016;  // 4 inch wheels
        double DRIVE_GEAR_RATIO = 6.12;         // L3 gearing for SDS MK4i
        double TURN_GEAR_RATIO =  287.0 / 11.0;   // MK4i turn ratio

        // Derived constants (do not modify)
        double WHEEL_BASE_METERS = Units.inchesToMeters(WHEEL_BASE_INCHES);
        double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);

        double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        double DRIVE_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
        double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_FACTOR / 60.0;

        // Max wheel speed in meters per second (theoretical)
        double MAX_MOTOR_RPM = 6380.0;  // Kraken X60 free speed
        double MAX_WHEEL_SPEED_MPS = (MAX_MOTOR_RPM / 60.0) * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

        // Kinematics - module positions relative to robot center
        SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),   // FL
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),  // FR
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),  // BL
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)  // BR
        );
        /** Maximum pose jump */
        DoubleSupplier maxPoseJumpFeet = pref("Limelight/MaxPoseJumpFeet", 1.0);
    }

//endregion

//region Vision ----------------------------------------------------------------

    /**
     * Configuration for the Limelight vision and pose estimation
     */
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
        Vector<N3> lowConfidence = VecBuilder.fill(2.0, 2.0, 9999999);
        Vector<N3> mediumConfidence = VecBuilder.fill(0.9, 0.9, 9999999);
        Vector<N3> highConfidence = VecBuilder.fill(0.5, 0.5, 9999999);

    }

    /**
     * Configuration for the Limelight simulation
     */
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

    /**
     * Configuration for PathPlanner autonomous path following.
     */
    interface PathPlanner {

        //=======================================================================
        // Robot physical properties (for path following)
        //=======================================================================

        /** Robot mass including bumpers and battery (kg) */
        double ROBOT_MASS_KG = 54.0;

        /** Robot moment of inertia (kg*m^2) - estimate or calculate from CAD */
        double ROBOT_MOI = 6.0;

        /** Wheel coefficient of friction */
        double WHEEL_COF = 1.0;

        //=======================================================================
        // Drive motor properties (Kraken X60)
        //=======================================================================

        /** Motor free speed (RPM) */
        double MOTOR_FREE_SPEED_RPM = 6380.0;

        /** Motor stall torque (N*m) */
        double MOTOR_STALL_TORQUE_NM = 7.09;

        /** Motor stall current (A) */
        double MOTOR_STALL_CURRENT_AMPS = 366.0;

        /** Current limit for drive motors (A) */
        double DRIVE_CURRENT_LIMIT_AMPS = 60.0;

        //=======================================================================
        // Path following PID tuning
        //=======================================================================

        /** Translation P gain for path following */
        DoubleSupplier translationKP = pref("PathPlanner/Translation/kP", 0.0);

        /** Translation I gain for path following */
        DoubleSupplier translationKI = pref("PathPlanner/Translation/kI", 0.0);

        /** Translation D gain for path following */
        DoubleSupplier translationKD = pref("PathPlanner/Translation/kD", 0.0);

        /** Rotation P gain for path following */
        DoubleSupplier rotationKP = pref("PathPlanner/Rotation/kP", 0.0);

        /** Rotation I gain for path following */
        DoubleSupplier rotationKI = pref("PathPlanner/Rotation/kI", 0.0);

        /** Rotation D gain for path following */
        DoubleSupplier rotationKD = pref("PathPlanner/Rotation/kD", 0.0);

        //=======================================================================
        // Path following settings
        //=======================================================================

        /** Enable PathPlanner logging to NetworkTables/AdvantageScope */
        BooleanSupplier enableLogging = pref("PathPlanner/Logging?", true);
    }

    /**
     * Configuration for the LED subsystem using REV Blinkin.
     * <p>
     * The Blinkin is controlled via PWM like a Spark motor controller.
     * Different "speed" values (-1.0 to 1.0) select colors and patterns.
     *
     * @see frc.robot.subsystems.led.LEDSignal
     */
    interface LED {

        /** PWM port for the REV Blinkin LED controller */
        int pwmPort = 0;

    }

    /**
     * Configuration for the {@link frc.robot.subsystems.shooter.ShooterSubsystem}
     * and related functionality
     */
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

    /*
     * Configuration for the front intake subsystem.
     * <p>
     * Uses closed-loop velocity control with separate intake/eject speeds.
     */
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

}

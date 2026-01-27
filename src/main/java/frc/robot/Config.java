package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    /**
     * Configuration for the Limelight vision and pose estimation.
     * <p>
     * Supports two estimation strategies:
     * <ul>
     *   <li>Classic (MegaTag1): Uses raw fiducial detection</li>
     *   <li>MegaTag2: Uses robot orientation for improved accuracy</li>
     * </ul>
     */
    interface Limelight {

        /** Name of the Limelight in NetworkTables */
        String limelightName = "limelight";

        //=======================================================================
        // Vision enable/strategy
        //=======================================================================

        /** Master enable/disable for vision pose estimation */
        BooleanSupplier visionEnabled = pref("Vision/Enabled?", true);

        /** Strategy: 0=Classic only, 1=MegaTag2 only, 2=Both (use best) */
        DoubleSupplier strategy = pref("Vision/Strategy", 1.0);

        //=======================================================================
        // Standard deviation tuning (trust levels)
        //=======================================================================

        /** Base XY standard deviation for single-tag estimates (meters) */
        DoubleSupplier baseStdDevXY_1Tag = pref("Vision/BaseStdDev/XY_1Tag", 0.9);

        /** Base XY standard deviation for multi-tag estimates (meters) */
        DoubleSupplier baseStdDevXY_MultiTag = pref("Vision/BaseStdDev/XY_MultiTag", 0.5);

        /** Distance (meters) at which standard deviation doubles */
        DoubleSupplier distScaleFactor = pref("Vision/DistScaleFactor", 3.0);

        //=======================================================================
        // Filtering thresholds
        //=======================================================================

        /** Maximum ambiguity allowed for single-tag estimates (0-1) */
        DoubleSupplier maxAmbiguity = pref("Vision/MaxAmbiguity", 0.7);

        /** Maximum distance to camera for trusted estimates (meters) */
        DoubleSupplier maxDistToCamera = pref("Vision/MaxDistToCamera", 4.0);

        /** Maximum yaw rate for MegaTag2 estimates (degrees per second) */
        DoubleSupplier maxYawRateDps = pref("Vision/MaxYawRateDPS", 720.0);

        /** Minimum tag area for MegaTag2 estimates (0-100% of image) */
        DoubleSupplier minTagArea = pref("Vision/MinTagArea", 0.1);

        /** Maximum allowed pose jump (meters) - rejects sudden large changes */
        DoubleSupplier maxPoseJump = pref("Vision/MaxPoseJump", 2.0);

        //=======================================================================
        // Field boundary (2024 Crescendo field dimensions)
        //=======================================================================

        /** Field length in meters (x-axis) */
        double FIELD_LENGTH_METERS = 16.54;

        /** Field width in meters (y-axis) */
        double FIELD_WIDTH_METERS = 8.21;

        /** Margin outside field boundaries to allow (meters) */
        double FIELD_BOUNDARY_MARGIN = 0.5;

        //=======================================================================
        // 2026 REBUILT AprilTag IDs by field element
        //=======================================================================

        /** Tower AprilTag IDs (central scoring/climbing structures) */
        int[] TOWER_TAG_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};

        /** Outpost AprilTag IDs (Tags 13-14 on one outpost, 29-30 on the other) */
        int[] OUTPOST_TAG_IDS = {13, 14, 29, 30};

        /** Hub/Alliance Wall AprilTag IDs */
        int[] HUB_TAG_IDS = {15, 16, 31, 32};

        //=======================================================================
        // 2026 REBUILT AprilTag IDs by field position
        //=======================================================================

        /** Left side of field (Blue alliance perspective) */
        int[] LEFT_SIDE_TAG_IDS = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

        /** Right side of field (Blue alliance perspective) */
        int[] RIGHT_SIDE_TAG_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

        /** Center field elements (both tower clusters) */
        int[] CENTER_TAG_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};

        /** Alliance wall tags (field edges) */
        int[] ALLIANCE_WALL_TAG_IDS = {13, 14, 15, 16, 29, 30, 31, 32};

        //=======================================================================
        // ProjectBucephalus-style standard deviation
        //=======================================================================

        /** Baseline linear stdDev (meters) at 1m distance with 1 tag */
        DoubleSupplier linearStdDevBaseline = pref("Vision/LinearStdDevBaseline", 0.08);
    }

    /**
     * Configuration for the Limelight simulation.
     * <p>
     * Simulates AprilTag detection based on robot position, camera FOV, and tag visibility.
     */
    interface LimelightSim {

        //=======================================================================
        // Camera mounting (relative to robot center, in meters)
        //=======================================================================

        /** Forward offset from robot center to camera (meters) */
        DoubleSupplier cameraForwardOffset = pref("LimelightSim/ForwardOffset", 0.3);

        /** Side offset from robot center to camera (positive = left, meters) */
        DoubleSupplier cameraSideOffset = pref("LimelightSim/SideOffset", 0.0);

        /** Camera height from ground (meters) */
        DoubleSupplier cameraHeight = pref("LimelightSim/Height", 0.5);

        /** Camera pitch angle (positive = tilted up, degrees) */
        DoubleSupplier cameraPitch = pref("LimelightSim/PitchDeg", 20.0);

        //=======================================================================
        // Camera FOV
        //=======================================================================

        /** Horizontal field of view (degrees) */
        DoubleSupplier horizontalFov = pref("LimelightSim/HorizontalFOV", 63.3);

        /** Vertical field of view (degrees) */
        DoubleSupplier verticalFov = pref("LimelightSim/VerticalFOV", 49.7);

        //=======================================================================
        // Detection parameters
        //=======================================================================

        /** Maximum distance to detect tags (meters) */
        DoubleSupplier maxDetectionDistance = pref("LimelightSim/MaxDistance", 5.0);

        /** Minimum distance to detect tags (meters) */
        DoubleSupplier minDetectionDistance = pref("LimelightSim/MinDistance", 0.3);

        /** Maximum angle between camera and tag normal for detection (degrees) */
        DoubleSupplier maxTagAngle = pref("LimelightSim/MaxTagAngle", 60.0);

        //=======================================================================
        // Noise and latency simulation
        //=======================================================================

        /** Standard deviation for position noise (meters) */
        DoubleSupplier positionNoiseStdDev = pref("LimelightSim/PositionNoise", 0.02);

        /** Standard deviation for rotation noise (degrees) */
        DoubleSupplier rotationNoiseStdDev = pref("LimelightSim/RotationNoise", 1.0);

        /** Simulated pipeline latency (milliseconds) */
        DoubleSupplier pipelineLatency = pref("LimelightSim/LatencyMs", 25.0);

        /** Probability of dropping a frame (0.0-1.0) */
        DoubleSupplier frameDropProbability = pref("LimelightSim/FrameDropProb", 0.05);

        //=======================================================================
        // Enable
        //=======================================================================

        /** Enable/disable the Limelight simulation */
        BooleanSupplier enabled = pref("LimelightSim/Enabled?", true);

        //=======================================================================
        // Constants
        //=======================================================================

        /** AprilTag size in meters (6.5 inches for 2024/2025) */
        double TAG_SIZE_METERS = 0.1651;
    }

    /**
     * Configuration for odometry diagnostics.
     */
    interface Odometry {

        /** Enable odometry diagnostics publishing to dashboard */
        BooleanSupplier enableDiagnostics = pref("Odometry/Diagnostics?", true);

        /** Drift threshold for warning (meters) - logged when exceeded */
        DoubleSupplier driftWarning = pref("Odometry/DriftWarning_m", 0.5);
    }

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
        int PWM_PORT = 0;

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

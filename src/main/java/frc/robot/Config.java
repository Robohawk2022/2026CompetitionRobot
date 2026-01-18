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

        /** Log swerve module states and PID values */
        BooleanSupplier swerveLogging = pref("Logging/Swerve?", false);

        /** Log vision estimates and rejection reasons */
        BooleanSupplier visionLogging = pref("Logging/Vision?", true);

        /** Log odometry diagnostics */
        BooleanSupplier odometryLogging = pref("Logging/Odometry?", true);

        /** Log shooter state and speeds */
        BooleanSupplier shooterLogging = pref("Logging/Shooter?", false);

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

    /**
     * Configuration for the swerve drive subsystem.
     * <p>
     * IMPORTANT: Update the placeholder values below for your specific robot!
     */
    interface Swerve {

        //=======================================================================
        // PLACEHOLDER VALUES - UPDATE THESE FOR YOUR ROBOT
        //=======================================================================

        // Chassis dimensions (inches) - measure from wheel center to wheel center
        double WHEEL_BASE_INCHES = 30.25;       // Front-to-back distance
        double TRACK_WIDTH_INCHES = 26.5;       // Left-to-right distance

        // CAN IDs - Module order: FL (Front Left), FR, BL, BR
        int FL_DRIVE_ID = 41;
        int FL_TURN_ID = 42;
        int FL_ENCODER_ID = 43;
        int FR_DRIVE_ID = 21;
        int FR_TURN_ID = 22;
        int FR_ENCODER_ID = 23;
        int BL_DRIVE_ID = 11;
        int BL_TURN_ID = 12;
        int BL_ENCODER_ID = 13;
        int BR_DRIVE_ID = 31;
        int BR_TURN_ID = 32;
        int BR_ENCODER_ID = 33;
        int PIGEON_ID = 10;

        // Module angular offsets (radians) - Calibrate by pointing all wheels forward
        // and reading the CANcoder values, then negate them here
        double FL_ANGULAR_OFFSET = 0;
        double FR_ANGULAR_OFFSET = 0;
        double BL_ANGULAR_OFFSET = 0;
        double BR_ANGULAR_OFFSET = 0;

        //=======================================================================
        // Tunable values (adjustable via dashboard/preferences)
        //=======================================================================

        DoubleSupplier maxSpeedFps = pref("Swerve/MaxSpeedFPS", 15.0);
        DoubleSupplier maxRotationDps = pref("Swerve/MaxRotationDPS", 360.0);

        // Drive motor PID (velocity control)
        DoubleSupplier driveKP = pref("Swerve/Drive/kP", 0.1);
        DoubleSupplier driveKV = pref("Swerve/Drive/kV", 0.12);
        DoubleSupplier driveKS = pref("Swerve/Drive/kS", 0.0);

        // Turn motor PID (position control)
        DoubleSupplier turnKP = pref("Swerve/Turn/kP", 0.0);
        DoubleSupplier turnKD = pref("Swerve/Turn/kD", 0.0);

        //=======================================================================
        // Physical constants (change only if hardware changes)
        //=======================================================================

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
     * Configuration for high-frequency odometry and diagnostics.
     */
    interface Odometry {

        /** High-frequency odometry update rate (Hz) */
        double HF_FREQUENCY = 250.0;

        /** Number of poses to keep in history (~200ms at 250Hz) */
        int POSE_HISTORY_SIZE = 50;

        /** Enable high-frequency odometry (CTRE synchronized reads) */
        BooleanSupplier enableHF = pref("Odometry/EnableHF?", true);

        /** Enable odometry diagnostics publishing to dashboard */
        BooleanSupplier enableDiagnostics = pref("Odometry/Diagnostics?", true);

        /** Drift threshold for warning (meters) - logged when exceeded */
        DoubleSupplier driftWarning = pref("Odometry/DriftWarning_m", 0.5);
    }

}

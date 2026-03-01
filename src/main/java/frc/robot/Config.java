package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

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

//region Swerve (Teleop) -------------------------------------------------------

    interface SwerveTeleop {

        /** Maximum speeds */
        DoubleSupplier maxTranslate = pref("SwerveTeleop/MaxTranslateFPS", 7.0);
        DoubleSupplier maxRotate = pref("SwerveTeleop/MaxRotateDPS", 180.0);
        DoubleSupplier maxOrbit = pref("SwerveTeleop/MaxOrbitFPS", 15.0);

        /** Speed mode factors (turbo/sniper) */
        DoubleSupplier sniperFactor = pref("SwerveTeleop/SniperFactor", 0.5);
        DoubleSupplier turboFactor = pref("SwerveTeleop/TurboFactor", 2.0);
        BooleanSupplier applySniperToRotation = pref("SwerveTeleop/SniperRotation?", true);

        /** Driver relative mode */
        BooleanSupplier driverRelative = pref("SwerveTeleop/DriverRelative?", false);

        /** Joystick deadband & exponent */
        DoubleSupplier deadband = pref("SwerveTeleop/Deadband", 0.1);
        DoubleSupplier exponent = pref("SwerveTeleop/Exponent", 2.0);

        /** Use XBox mapping */
        BooleanSupplier useXboxMapping = pref("SwerveTeleop/UseXboxMapping?", true);

    }

//endregion

//region Swerve (Auto) ---------------------------------------------------------

    interface SwerveAuto {

        /** Speed for opening hopper */
        DoubleSupplier openHopperVelocity = pref("SwerveAuto/OpenHopperFPS", 10.0);
        DoubleSupplier openHopperSecs = pref("SwerveAuto/OpenHopperSecs", 0.5);

        /** Maximum translation velocity in feet per second */
        DoubleSupplier maxTranslationVelocity = pref("SwerveAuto/MaxTranslationFPS", 7.0);

        /** Maximum translation acceleration in feet per second squared */
        DoubleSupplier maxTranslationAcceleration = pref("SwerveAuto/MaxTranslationFPSS", 7.0);

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

        /** Gains for path following */
        DoubleSupplier translationP = pref("PathPlanner/Translation/kP", 10.0);
        DoubleSupplier rotationP = pref("PathPlanner/Rotation/kP", 7.0);

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

        /** Maximum pose jump (vision corrections larger than this are rejected) */
        DoubleSupplier maxPoseJumpFeet = pref("Limelight/MaxPoseJumpFeet", 2.0);
    }

//endregion

//region Shared helpers --------------------------------------------------------

    class PIDFConfig {

        public final DoubleSupplier p;
        public final DoubleSupplier i;
        public final DoubleSupplier iz;
        public final DoubleSupplier d;
        public final DoubleSupplier v;

        public PIDFConfig(String name,
                          double defaultP,
                          double defaultI,
                          double defaultIZone,
                          double defaultD,
                          double defaultV) {
            p = pref(name+"/kP", defaultP);
            i = pref(name+"/kI", defaultI);
            iz = pref(name+"/kIZone", defaultIZone);
            d = pref(name+"/kD", defaultD);
            v = pref(name+"/kV", defaultV);
        }

    }

    class BallPathSpeeds {

        public final DoubleSupplier intakeRpm;
        public final DoubleSupplier feederRpm;
        public final DoubleSupplier agitatorRpm;

        public BallPathSpeeds(String path,
                              double defaultIntakeRpm,
                              double defaultFeederRpm,
                              double defaultAgitatorRpm) {
            intakeRpm = pref(path+"/IntakeRpm", defaultIntakeRpm);
            feederRpm = pref(path+"/FeederRpm", defaultFeederRpm);
            agitatorRpm = pref(path+"/AgitatorRpm", defaultAgitatorRpm);
        }
    }

//endregion

//region Shooter ---------------------------------------------------------------

    interface Shooter {

        /** PID configuration for the shooter motor */
        PIDFConfig shooterPid = new PIDFConfig("BallPID/ShooterMotor", 0.0005, 0.0, 0.0, 0.0, 0.0002);

        /** Target RPM for shooting */
        DoubleSupplier shootRpm = pref("ShootMode/ShooterRpm", 3600.0);
        DoubleSupplier intakeRpm = pref("IntakeMode/ShooterRpm", 1000.0);

        /** How close to target RPM counts as "at speed" */
        DoubleSupplier shootSpeedTolerance = pref("ShootMode/ShootSpeedTolerance", 50.0);
        DoubleSupplier shootDistanceFeet = pref("ShootMode/ShootDistance", 7.0);

        /** Stall detection */
        DoubleSupplier stallSpeed = () -> 3.0; // pref("Shooter/StallSpeed", 30.0);
        DoubleSupplier stallTime = () -> 1.0; // pref("Shooter/StallTime", 1.0);
    }

//endregion

//region BallPath --------------------------------------------------------------

    interface BallPath {

        /** PID configuration for ball-path motors */
        PIDFConfig intakePid = new PIDFConfig("BallPID/IntakeMotor", 0.00001, 0.0, 0.0, 0.0, 0.00017);
        PIDFConfig feederPid = new PIDFConfig("BallPID/FeederMotor", 0.00001, 0.0, 0.0, 0.0, 0.00017);
        PIDFConfig agitatorPid = new PIDFConfig("BallPID/AgitatorMotor", 0.00001, 0.0, 0.0, 0.0, 0.00017);

        /** Target speeds for various modes */
        BallPathSpeeds intakeSpeeds = new BallPathSpeeds("IntakeMode", 3000.0, 3000.0, 0.0);
        BallPathSpeeds ejectSpeeds = new BallPathSpeeds("EjectMode", 3000.0, 3000.0, 0.0);
        BallPathSpeeds feedSpeeds = new BallPathSpeeds("ShootMode", 4000.0, 4000.0, 4000.0);

        /** Stall detection */
        DoubleSupplier stallSpeed = () -> 30.0; // pref("BallPathSubsystem/StallSpeed", 30.0);
        DoubleSupplier stallTime = () -> 1.0; // pref("BallPathSubsystem/StallTime", 1.0);
    }

//endregion

}

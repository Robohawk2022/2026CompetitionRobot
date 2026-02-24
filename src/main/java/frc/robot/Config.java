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

//region Launcher --------------------------------------------------------------

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

    class LauncherSpeeds {

        public final DoubleSupplier intakeRpm;
        public final DoubleSupplier feederRpm;
        public final DoubleSupplier agitatorRpm;
        public final DoubleSupplier shooterRpm;

        public LauncherSpeeds(String path,
                              double defaultIntakeRpm,
                              double defaultFeederRpm,
                              double defaultAgitatorRpm,
                              double defaultShooterRpm) {
            intakeRpm = pref(path+"/IntakeRpm", defaultIntakeRpm);
            feederRpm = pref(path+"/FeederRpm", defaultFeederRpm);
            agitatorRpm = pref(path+"/AgitatorRpm", defaultAgitatorRpm);
            shooterRpm = pref(path+"/ShooterRpm", defaultShooterRpm);
        }
    }

    interface Launcher {

        /** PID configuration for motors */
        PIDFConfig intakePid = new PIDFConfig("LauncherSubsystem/IntakeMotor", 0.0004, 0.0, 20.0, 0.0, 0.00017);
        PIDFConfig feederPid = new PIDFConfig("LauncherSubsystem/FeederMotor", 0.0004, 0.0, 20.0, 0.0, 0.00017);
        PIDFConfig agitatorPid = new PIDFConfig("LauncherSubsystem/AgitatorMotor", 0.0004, 0.0, 20.0, 0.0, 0.00017);
        PIDFConfig shooterPid = new PIDFConfig("LauncherSubsystem/ShooterMotor", 0.0005, 0.0, 20.0, 0.0, 0.0002);

        /** Target speeds for various mode */
        LauncherSpeeds intakeSpeeds = new LauncherSpeeds("BallHandling/IntakeSpeeds", 3000.0, 3000.0, 0.0, 1000.0);
        LauncherSpeeds ejectSpeeds = new LauncherSpeeds("BallHandling/EjectSpeeds", 3000.0, 3000.0, 0.0, 1000.0);
        LauncherSpeeds shootSpeeds = new LauncherSpeeds("BallHandling/ShootSpeeds", 3000.0, 3000.0, 1000.0, 3000.0);

        /** Distance & spin up time for shooting */
        DoubleSupplier shootDistanceFeet = pref("BallHandling/ShootDistance", 7.0);
        DoubleSupplier shootDistanceTolerance = pref("BallHandling/ShootDistanceTolerance", 0.5);
        DoubleSupplier shootSpinupTime = pref("BallHandling/ShootSpinupTime", 1.0);
        DoubleSupplier shootSpeedTolerance = pref("BallHandling/ShootSpeedTolerance", 100.0);
        DoubleSupplier stallSpeed = pref("BallHandling/StallSpeed", 30.0);
        DoubleSupplier stallTime = pref("BallHandling/StallTime", 1.0);

    }

//endregion

}

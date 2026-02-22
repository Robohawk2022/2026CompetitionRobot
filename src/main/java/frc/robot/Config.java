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

//region Shooting --------------------------------------------------------------

    /**
     * Configuration for shooting distance presets.
     * <p>
     * Each distance has a target RPM for the shooter. The robot LED turns
     * green when within tolerance of a shooting distance, red otherwise.
     */
    interface Shooting {

        /** Close shooting distance in feet */
        DoubleSupplier closeDistance = pref("Shooting/CloseDistanceFt", 3.0);

        /** Shooter RPM for close distance */
        DoubleSupplier closeRPM = pref("Shooting/CloseRPM", 2525.0);

        /** Far shooting distance in feet */
        DoubleSupplier farDistance = pref("Shooting/FarDistanceFt", 4.5);

        /** Shooter RPM for far distance */
        DoubleSupplier farRPM = pref("Shooting/FarRPM", 3225.0);

        /** How close to the target distance we need to be (in feet) */
        DoubleSupplier distanceTolerance = pref("Shooting/DistanceToleranceFt", 0.5);

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

//region Launcher --------------------------------------------------------------

    interface Launcher {

        //=======================================================================
        // CAN IDs
        //=======================================================================

        int FEEDER_LEFT_CAN_ID = 32;
        int FEEDER_RIGHT_CAN_ID = 11;
        int SHOOTER_CAN_ID = 60;
        int SHOOTER_INTAKE_CAN_ID = 9;

        //=======================================================================
        // Gear ratios (for reference)
        //=======================================================================

        /** 4:1 * 3:1 = 12:1 total gear reduction per feeder */
        double FEEDER_GEAR_RATIO = 12.0;

        /** Shooter: dual NEO through 1:1 gearbox (no reduction) */
        double SHOOTER_GEAR_RATIO = 1.0;

        //=======================================================================
        // Feeder Left velocity PID - closed loop (SparkMax onboard)
        //=======================================================================

        DoubleSupplier feederLeftKV = pref("Launcher/FeederLeft/kV", 0.00017);
        DoubleSupplier feederLeftKP = pref("Launcher/FeederLeft/kP", 0.0004);
        DoubleSupplier feederLeftKD = pref("Launcher/FeederLeft/kD", 0.0);

        //=======================================================================
        // Feeder Right velocity PID - closed loop (SparkMax onboard)
        //=======================================================================

        DoubleSupplier feederRightKV = pref("Launcher/FeederRight/kV", 0.000175);
        DoubleSupplier feederRightKP = pref("Launcher/FeederRight/kP", 0.0004);
        DoubleSupplier feederRightKD = pref("Launcher/FeederRight/kD", 0.0);

        //=======================================================================
        // Shooter velocity PID - closed loop (SparkMax onboard)
        //=======================================================================

        DoubleSupplier shooterKV = pref("Launcher/Shooter/kV", 0.0002);
        DoubleSupplier shooterKP = pref("Launcher/Shooter/kP", 0.0005);
        DoubleSupplier shooterKD = pref("Launcher/Shooter/kD", 0.0);

        //=======================================================================
        // Shooter Intake velocity PID - closed loop (SparkMax onboard)
        //=======================================================================

        DoubleSupplier shooterIntakeKV = pref("Launcher/ShooterIntake/kV", 0.0002);
        DoubleSupplier shooterIntakeKP = pref("Launcher/ShooterIntake/kP", 0.0005);

        //=======================================================================
        // RPM targets — intake mode
        //=======================================================================

        /** Feeder Left and Right RPM during intake (intake motor off during intake) */
        DoubleSupplier feederRPM = pref("Launcher/FeederRPM", 3600.0);

        //=======================================================================
        // RPM targets — shoot mode
        //=======================================================================

        /** Feeder RPM when feeding balls to shooter during a shot */
        DoubleSupplier feedShootRPM = pref("Launcher/FeedShootRPM", 3600.0);

        /** Shooter (main flywheel) RPM during shooting */
        DoubleSupplier shooterRPM = pref("Launcher/ShooterRPM", 2525.0);
        DoubleSupplier shooterIntakingRPM = pref("Launcher/ShooterIntakingRPM", 1000.0);

        /** Intake motor (CAN 9) RPM during shooting (feeds ball into shooter) */
        DoubleSupplier intakeRPM = pref("Launcher/IntakeRPM", 3000.0);

        //=======================================================================
        // Motor configuration
        //=======================================================================

        boolean FEEDER_LEFT_INVERTED = true;
        boolean FEEDER_RIGHT_INVERTED = false;
        boolean SHOOTER_INVERTED = false;
        boolean SHOOTER_INTAKE_INVERTED = true;

        /** Current limit for all launcher motors in amps (NEO safe range: 40-60A) */
        DoubleSupplier currentLimit = pref("Launcher/CurrentLimit", 60.0);

        /** RPM tolerance for "at speed" check */
        DoubleSupplier tolerance = pref("Launcher/ToleranceRPM", 100.0);

    }

//endregion

}

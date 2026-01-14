package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class Util {

//region Constants -------------------------------------------------------------

    /** Maximum volts for motors */
    public static final double MAX_VOLTS = 12.0;

    /** Time slice between calls to periodic() */
    public static final double DT = 0.02;

    /** Heading with no value */
    public static final Rotation2d NAN_ROTATION = new Rotation2d(Double.NaN);

    /** Pose with no values */
    public static final Pose2d NAN_POSE = new Pose2d(Double.NaN, Double.NaN, NAN_ROTATION);

    /** Speed with no values */
    public static final ChassisSpeeds NAN_SPEED = new ChassisSpeeds(Double.NaN, Double.NaN, Double.NaN);

    /** Speed with 0 values */
    public static final ChassisSpeeds ZERO_SPEED = new ChassisSpeeds(0.0, 0.0, 0.0);

    /** State with 0 values */
    public static final State ZERO_STATE = new State(0.0, 0.0);

    /** State with no values */
    public static final State NAN_STATE = new State(Double.NaN, Double.NaN);

//endregion

//region Miscellaneous math ----------------------------------------------------

    /**
     * @param volts a voltage value
     * @return the supplied value, clamped to +/- 12 volts
     */
    public static double clampVolts(double volts) {
        return MathUtil.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
    }

    /**
     * @param value a value
     * @param limit supplier for the limit
     * @return the supplied value, clamped within +/- the limit value
     */
    public static double applyClamp(double value, DoubleSupplier limit) {
        double lim = limit.getAsDouble();
        return MathUtil.clamp(value, -lim, lim);
    }

    /**
     * @param value a value
     * @param lower supplier for the lower limit
     * @param upper supplier for the upper limit
     * @return the supplied value, clamped within the supplied limits
     */
    public static double applyClamp(double value, DoubleSupplier lower, DoubleSupplier upper) {
        double lo = lower.getAsDouble();
        double hi = upper.getAsDouble();
        return MathUtil.clamp(value, lo, hi);
    }

    /**
     * Adjusts the supplied {@link ChassisSpeeds} to keep them under a
     * specified speed limit. Translation speed is determined by combining
     * X and Y movement; if it needs to be scaled down we do so proportionally.
     *
     * @param speeds chassis speeds
     * @param maxTranslate maximum translation speed in feet per second
     * @param maxRotate maximum rotation speed in degrees per second
     */
    public static void clampFeetAndDegrees(ChassisSpeeds speeds,
                                           DoubleSupplier maxTranslate,
                                           DoubleSupplier maxRotate) {

        double mt = maxTranslate.getAsDouble();

        // calculate target speed in feet per second; if it's >max
        // we will want to scale the x and y speeds down
        double fps = Units.metersToFeet(Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond));
        if (fps > mt) {
            double factor = mt / fps;
            speeds.vxMetersPerSecond *= factor;
            speeds.vyMetersPerSecond *= factor;
        }

        // clamp the speed (making sure to translate the max value into
        // radians b/c that's what's in ChassisSpeeds
        speeds.omegaRadiansPerSecond = applyClamp(
                speeds.omegaRadiansPerSecond,
                () -> Units.degreesToRadians(maxRotate.getAsDouble()));
    }

    /**
     * @param degrees an angle in degrees
     * @return the supplied angle wrapped to (-180, 180)
     */
    public static double degreeModulus(double degrees) {
        return MathUtil.inputModulus(degrees, -180.0, 180.0);
    }

    /**
     * @param value a value
     * @param tolerance supplier for the tolerance
     * @return true if the supplied value is close enough to zero
     */
    public static boolean nearZero(double value, DoubleSupplier tolerance) {
        return Math.abs(value) < tolerance.getAsDouble();
    }

    /**
     * @param linearSpeed a linear speed
     * @param gearRatio the gear ratio of the motor (>1 means reduction)
     * @param wheelDiameter diameter of the wheel in the same units as speed
     * @return the motor speed in revolutions per second that will result in
     * the specified linear speed at the surface of the wheel
     */
    public static double linearSpeedToMotorSpeed(double linearSpeed,
                                                 double gearRatio,
                                                 double wheelDiameter) {
        return (linearSpeed / (Math.PI * wheelDiameter)) * gearRatio;
    }

    /**
     * @param motorSpeed a motor speed in revolutions per second
     * @param gearRatio the gear ratio of the motor (>1 means reduction)
     * @param wheelDiameter diameter of the wheel in the desired units
     * @return the linear speed in units per second that will be achieved when
     * the motor rotates at the given speed
     */
    public static double motorSpeedToLinearSpeed(double motorSpeed,
                                                 double gearRatio,
                                                 double wheelDiameter) {
        return (motorSpeed / gearRatio) * Math.PI * wheelDiameter;
    }

//endregion

//region AprilTag information --------------------------------------------------

    /* Indicates which field layout we'll use when loading information */
    static AprilTagFields useTheseFields = AprilTagFields.kDefaultField;

    /* Loaded field layout */
    static AprilTagFieldLayout fieldLayout = null;

    /* Mapping of tag ID to tag */
    static Map<Integer, AprilTag> allTags = new HashMap<>();

    /* Mapping of tag ID to tag pose */
    static Map<Integer,Pose2d> allPoses = new HashMap<>();

    /* Have we loaded the field layour yet? */
    static boolean initialized = false;

    /*
     * If we haven't yet been initialized, this will load the field layout
     * and precompute the maps of tags and tag poses. This can be expensive
     * so we only want to do it once.
     */
    static void initialize() {
        if (!initialized) {
            fieldLayout = AprilTagFieldLayout.loadField(useTheseFields);
            for (AprilTag tag : fieldLayout.getTags()) {
                allTags.put(tag.ID, tag);
                allPoses.put(tag.ID, tag.pose.toPose2d());
            }
            initialized = true;
        }
    }

    /**
     * In 2025 there was an issue where the field measurements were
     * different depending on which vendor made the field; if this
     * comes up again you should call this at the beginning of
     * {@link frc.robot.Main} to indicate which field layout to use.
     *
     * @param layout which field to use (null means default field)
     */
    public static void setFieldLayout(AprilTagFields layout) {
        if (layout == null) {
            layout = AprilTagFields.kDefaultField;
        }
        useTheseFields = layout;
    }

    /**
     * @return the current {@link AprilTagFieldLayout}
     */
    public static AprilTagFieldLayout getFieldLayout() {
        initialize();
        return fieldLayout;
    }

    /**
     * @param id an AprilTag ID
     * @return the supplied tag (null if it doesn't exist)
     */
    public static AprilTag getTag(int id) {
        initialize();
        return allTags.get(id);
    }

    /**
     * @param id an AprilTag ID
     * @return the pose of the supplied tag (null if it doesn't exist)
     */
    public static Pose2d getTagPose(int id) {
        initialize();
        return allPoses.get(id);
    }

    /**
     * @param tagId an AprilTag ID
     * @param offsetFeet how many feet in front of the tag you want to be
     * @return a pose facing the tag, offset by the amount specified; null if
     * the tag is invalid
     */
    public static Pose2d getPoseFacingTag(int tagId, double offsetFeet) {

        Pose2d tagPose = Util.getTagPose(tagId);
        if (tagPose == null) {
            return null;
        }

        // +X puts us in front of the tag, the rotation turns us around to
        // face the tag
        Transform2d transform = new Transform2d(
                new Translation2d(Units.feetToMeters(offsetFeet), 0.0),
                Rotation2d.k180deg);

        return tagPose.transformBy(transform);
    }

    /**
     * @param currentPose the robot's current pose
     * @param shouldConsider a test for tags; if it returns false a tag is ignore
     * @return the {@link AprilTag} closest to the robot that meets the
     * specified criteria
     */
    public static AprilTag closestTagFilteredBy(Pose2d currentPose,
                                                Predicate<AprilTag> shouldConsider) {

        initialize();

        // this will be our closest matching tag when we're done
        AprilTag closestTag = null;
        double closestDistance = Double.POSITIVE_INFINITY;

        for (AprilTag thisTag : allTags.values()) {

            // if we already have a closer tag, ignore this one
            double thisDistance = feetBetween(thisTag.pose.toPose2d(), currentPose);
            if (thisDistance > closestDistance) {
                continue;
            }

            // if this tag doesn't match the test, ignore it and continue
            if (!shouldConsider.test(thisTag)) {
                continue;
            }

            // looks like this is better than whatever we had
            closestTag = thisTag;
            closestDistance = thisDistance;
        }

        return closestTag;
    }

//endregion

//region Match information -----------------------------------------------------

    /* Are we the red alliance? */
    static BooleanEntry isRedAlliance = null;

    /**
     * Fetching the alliance is different in simulation versus the real
     * game. In simulation, we want to fetch it from the dashboard; in the
     * real game we'll talk to the driver station.
     *
     * @return true if we are on the red alliance?
     */
    public static boolean isRedAlliance() {
        if (RobotBase.isSimulation()) {
            if (isRedAlliance == null) {
                isRedAlliance = NetworkTableInstance.getDefault()
                        .getTable("FMSInfo")
                        .getBooleanTopic("IsRedAlliance")
                        .getEntry(false);
            }
            return isRedAlliance.get();
        } else {
            return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        }
    }

    /**
     * @return the opposite of {@link #isRedAlliance()}
     */
    public static boolean isBlueAlliance() {
        return !isRedAlliance();
    }

    /**
     * @return are we in competition?
     */
    public static boolean isCompetition() {
        return DriverStation.isFMSAttached();
    }

//endregion

//region Pose publishing -------------------------------------------------------

    /* Loggers for logging pose structs */
    static final Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();

    /* Prefix for logging pose structs */
    static String poseLoggingPrefix = "SmartDashboard/SwervePoses/";

    /**
     * Changes the prefix for publishing fields to the dashboard. You should
     * call this at the beginning of {@link frc.robot.Main} if you are going
     * to change it.
     *
     * @param prefix the prefix to use
     */
    public static void setPoseLoggingPrefix(String prefix) {
        poseLoggingPrefix = prefix;
    }

    /**
     * Publish a pose to the dashboard
     *
     * @param key the key to publish under (this automatically adds the
     *            {@link #poseLoggingPrefix} so it will show up under the
     *           correct topic in the dashboard)
     * @param val the pose to publish (if null, we will use {@link Util#NAN_POSE
     */
    public static void publishPose(String key, Pose2d val) {

        if (val == null) {
            val = Util.NAN_POSE;
        }

        // see if a publisher already exists
        StructPublisher<Pose2d> publisher = posePublishers.get(key);

        // create it if it doesn't (we add the SmartDashboard prefix so
        // it shows up next to other values we publish)
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                    .getStructTopic(poseLoggingPrefix+key, Pose2d.struct)
                    .publish();
            posePublishers.put(key, publisher);
        }

        publisher.set(val);
    }

//endregion

//region Speed math ------------------------------------------------------------

    /**
     * What we usually call "field relative" is actually "driver relative".
     * Converting speeds from driver relative to robot relative involves
     * two translations - one based on the driver's POV and another based on
     * how the robot is oriented.
     *
     * @param driverRelativeSpeeds speeds relative to the driver
     * @return speeds relative to the robot
     */
    public static ChassisSpeeds fromDriverRelativeSpeeds(
            ChassisSpeeds driverRelativeSpeeds,
            Rotation2d robotHeading) {

        // incoming speeds are interpreted like so:
        //   +X goes away from the driver
        //   +Y goes to the driver's left
        ChassisSpeeds fieldRelativeSpeeds;

        // if the driver is on the blue alliance, they are looking
        // at the field "normally" - so going away from them is
        // also +X on the field
        if (isBlueAlliance()) {
            fieldRelativeSpeeds = driverRelativeSpeeds;
        }

        // if they're red, they are actually looking at the field from
        // the opposite side (so going away from them is -X on the
        // field, and to their left is -Y)
        else {
            fieldRelativeSpeeds = new ChassisSpeeds(
                    -driverRelativeSpeeds.vxMetersPerSecond,
                    -driverRelativeSpeeds.vyMetersPerSecond,
                    driverRelativeSpeeds.omegaRadiansPerSecond);
        }

        // to get to fully robot-relative speeds, we need to consider
        // the current heading of the robot
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                robotHeading);
    }

    /**
     * @return true if the supplied speeds include an XY translation
     * greater than the supplied tolerance
     */
    public static boolean isTranslatingMoreThan(ChassisSpeeds speeds,
                                                DoubleSupplier tolerance) {
        double hyp = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return MathUtil.isNear(
                0.0,
                Units.metersToFeet(hyp),
                tolerance.getAsDouble());
    }

    /** @return true if the supplied speeds include a rotation */
    public static boolean isRotating(ChassisSpeeds speeds) {
        return Math.abs(speeds.omegaRadiansPerSecond) > 0.0;
    }

//endregion

//region Pose math -------------------------------------------------------------

    /**
     * @param start a starting pose
     * @param relativeTarget a target pose, in the coordinate system of the
     *                       robot (for example: x=1.0, y=0.0, r=180d means
     *                       the final pose will be one meter in front of the
     *                       start pose and facing the opposite direction)
     * @return the calculated final pose
     */
    public static Pose2d addRelativePose(Pose2d start, Pose2d relativeTarget) {
        Transform2d transform2d = new Transform2d(
                relativeTarget.getTranslation(),
                relativeTarget.getRotation());
        return start.plus(transform2d);
    }

    /**
     * @return the distance between the two poses in meters
     */
    public static double metersBetween(Pose2d start, Pose2d end) {
        return start.minus(end).getTranslation().getNorm();
    }

    /**
     * @return the distance between the two poses in feet
     */
    public static double feetBetween(Pose2d start, Pose2d end) {
        return Units.metersToFeet(metersBetween(start, end));
    }

    /**
     * Calculates the rotation between poses. The sign of this rotation may be
     * positive or negative depending on whether we're going left or right. It
     * should represent the "shortest way around".
     *
     * @param startPose the starting pose
     * @param finalPose the ending pose
     * @return the rotation between the two poses in degrees
     */
    public static double degreesBetween(Pose2d startPose, Pose2d finalPose) {
        return Units.radiansToDegrees(radiansBetween(startPose, finalPose));
    }

    /**
     * Calculates the rotation between poses. The sign of this rotation may be
     * positive or negative depending on whether we're going left or right. It
     * should represent the "shortest way around".
     *
     * @param startPose the starting pose
     * @param finalPose the ending pose
     * @return the rotation between the two poses in radians
     */
    public static double radiansBetween(Pose2d startPose, Pose2d finalPose) {
        return finalPose.getRotation()
                .minus(startPose.getRotation())
                .getRadians();
    }

    /**
     * Adds the specified {@link ChassisSpeeds} to the specified {@link Pose2d},
     * scaled by {@link #DT}
     * @param pose original pose
     * @param speeds speed of travel
     * @return the updated pose
     */
    public static Pose2d incrementPose(Pose2d pose, ChassisSpeeds speeds) {
        return new Pose2d(
                pose.getX() + speeds.vxMetersPerSecond * Util.DT,
                pose.getY() + speeds.vyMetersPerSecond * Util.DT,
                pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Util.DT)));
    }

//endregion

//region Logging & dashboard ---------------------------------------------------

    /**
     * Formats & writes a message to Rio log after formatting (adds a
     * newline to the end of the message if there isn't one).
     *
     * @param message the text of the message
     * @param args arguments to the message
     */
    public static void log(String message, Object... args) {
        System.out.printf(message, args);
        if (!message.endsWith("%n")) {
            System.out.println();
        }
    }

//endregion

//region Preferences -----------------------------------------------------------

    /**
     * Creates a boolean-valued preference. Preferences are persistent values
     * in NetworkTables. The supplied default value will be used at the start;
     * if someone changes the value (e.g. from the dashboard), that new value
     * will be used and will persist across restarts. See the
     * <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/robot-preferences.html">documentation</a>
     * for more information.
     *
     * @param name the name for the preference
     * @param defaultValue the default value of the preference
     * @return a {@link BooleanSupplier} providing access to the current value
     * of the preference, reflecting subsequent updates via the dashboard
     */
    public static BooleanSupplier pref(String name, boolean defaultValue) {
        Preferences.initBoolean(name, defaultValue);
        return () -> Preferences.getBoolean(name, defaultValue);
    }

    /**
     * Creates a double-valued preference. Preferences are persistent values
     * in NetworkTables. The supplied default value will be used at the start;
     * if someone changes the value (e.g. from the dashboard), that new value
     * will be used and will persist across restarts. See the
     * <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/robot-preferences.html">documentation</a>
     * for more information.
     *
     * @param name the name for the preference
     * @param defaultValue the default value of the preference
     * @return a {@link DoubleSupplier} providing access to the current value
     * of the preference, reflecting subsequent updates via the dashboard
     */
    public static DoubleSupplier pref(String name, double defaultValue) {
        Preferences.initDouble(name, defaultValue);
        return () -> Preferences.getDouble(name, defaultValue);
    }

//endregion

}
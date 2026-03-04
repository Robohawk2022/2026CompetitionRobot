package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;

/**
 * Utility class for field-related operations including AprilTag lookups,
 * field boundary checks, and game-specific locations.
 */
public class Field {

//region Field layout ----------------------------------------------------------

    /** Indicates which field layout we'll use when loading information */
    static AprilTagFields useTheseFields = AprilTagFields.kDefaultField;

    /** Loaded field layout */
    static AprilTagFieldLayout fieldLayout = null;

    /** Mapping of tag ID to tag */
    static Map<Integer, AprilTag> allTags = new HashMap<>();

    /** Mapping of tag ID to tag pose */
    static Map<Integer, Pose2d> allPoses = new HashMap<>();

    /** Have we loaded the field layout yet? */
    static boolean initialized = false;

    /**
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

//endregion

//region AprilTag lookups ------------------------------------------------------

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

        Pose2d tagPose = getTagPose(tagId);
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
     * @param shouldConsider a test for tags; if it returns false a tag is ignored
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
            double thisDistance = Util.feetBetween(thisTag.pose.toPose2d(), currentPose);
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

//region Field boundaries ------------------------------------------------------

    /**
     * @param pose a pose
     * @return does this pose lie outside the field boundaries
     */
    public static boolean isOutsideField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        if (x < 0.0 || x > getFieldLayout().getFieldLength()) {
            return true;
        }
        return (y < 0 || y > getFieldLayout().getFieldWidth());
    }

//endregion

//region Game-specific locations -----------------------------------------------

    /**
     * Represents one of the sides of the hub. Each side knows which AprilTags
     * may be mounted on it.
     */
    public enum HubSide {

        NORTH(Set.of(5, 8, 18, 27)),
        SOUTH(Set.of(2, 11, 21, 24)),
        NEUTRAL(Set.of(3, 4, 19, 20)),
        ALLIANCE(Set.of(9, 10, 25, 26));

        public final Set<Integer> tags;

        HubSide(Set<Integer> tags) {
            this.tags = tags;
        }
    }

    static Pose2d blueHubCenter = null;
    static Pose2d redHubCenter = null;

    /**
     * @return the pose at the center of the hub for the current alliance
     */
    public static Pose2d getHubCenter() {
        return getHubCenter(Util.isRedAlliance());
    }

    /**
     * Returns the center of the hub (tower) for the specified alliance.
     * For 2026, the hub center is calculated as the midpoint between two
     * AprilTags on opposite sides of the hub:
     * <ul>
     *   <li>Red alliance: tags 2 and 5</li>
     *   <li>Blue alliance: tags 18 and 21</li>
     * </ul>
     *
     * @return the pose at the center of the hub for the specified alliance
     */
    public static Pose2d getHubCenter(boolean red) {

        Pose2d tag1;
        Pose2d tag2;
        if (red) {
            if (redHubCenter != null) {
                return redHubCenter;
            }
            tag1 = getTagPose(2);
            tag2 = getTagPose(5);
        } else {
            if (blueHubCenter != null) {
                return blueHubCenter;
            }
            tag1 = getTagPose(18);
            tag2 = getTagPose(21);
        }

        Pose2d center = new Pose2d(
                (tag1.getX() + tag2.getX()) / 2.0,
                (tag1.getY() + tag2.getY()) / 2.0,
                Rotation2d.kZero);

        if (red) {
            redHubCenter = center;
        } else {
            blueHubCenter = center;
        }

        return center;
    }

    /**
     * @return which wall of the arena does the supplied hub tag face;
     * null if it is not a hub tag
     */
    public static HubSide getHubSide(int id) {
        for (HubSide side : HubSide.values()) {
            if (side.tags.contains(id)) {
                return side;
            }
        }
        return null;
    }

    /**
     * @return true if the supplied pose on the "north" side of the field,
     * false otherwise
     */
    public static boolean isNorthSide(Pose2d pose) {
        return Units.metersToInches(pose.getY()) < 158.84;
    }

    /**
     * @return opposite of {@link #isNorthSide(Pose2d)}
     */
    public static boolean isSouthSide(Pose2d pose) {
        return !isNorthSide(pose);
    }

    /**
     * @return if the supplied pose is in the neutral zone; false otherwise
     */
    public static boolean isNeutralZone(Pose2d pose) {
        return Units.metersToInches(pose.getX()) > 182.11
                && Units.metersToInches(pose.getX()) < 469.11;
    }

    /**
     * @return opposite of {@link #isNeutralZone(Pose2d)}
     */
    public static boolean isAllianceZone(Pose2d pose) {
        return !isNeutralZone(pose);
    }

//endregion

}

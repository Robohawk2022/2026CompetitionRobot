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
     * Returns the center of the hub (tower) for the current alliance.
     * <p>
     * For 2026, the hub center is calculated as the midpoint between two
     * AprilTags on opposite sides of the hub:
     * <ul>
     *   <li>Red alliance: tags 2 and 5</li>
     *   <li>Blue alliance: tags 18 and 21</li>
     * </ul>
     *
     * @return the pose at the center of the hub for the current alliance
     */
    public static Pose2d getHubCenter() {

        Pose2d tag1;
        Pose2d tag2;

        if (Util.isRedAlliance()) {
            tag1 = getTagPose(2);
            tag2 = getTagPose(5);
        } else {
            tag1 = getTagPose(18);
            tag2 = getTagPose(21);
        }

        return new Pose2d(
                (tag1.getX() + tag2.getX()) / 2.0,
                (tag1.getY() + tag2.getY()) / 2.0,
                Rotation2d.kZero);
    }

//endregion

}

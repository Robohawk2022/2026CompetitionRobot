package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.util.Field;

/**
 * Represents information about the current in-view target for the Limelight
 * (note that this is visual targeting, not pose estimation)
 */
public class LimelightTarget {

    double area = Double.NaN;
    double horizontalOffset = Double.NaN;
    double verticalOffset = Double.NaN;
    int tagId = -1;
    Pose2d tagPose = null;
    boolean valid = false;

    public void addToDashboard(SendableBuilder builder) {
        builder.addDoubleProperty("Targeting/Area", this::getArea, null);
        builder.addDoubleProperty("Targeting/HorizontalOffset", this::getHorizontalOffset, null);
        builder.addDoubleProperty("Targeting/VerticalOffset", this::getVerticalOffset, null);
        builder.addIntegerProperty("Targeting/DetectedTagId", this::getTagId, null);
    }

    /**
     * Invalidates the estimate (clears area/offset)
     */
    public void invalidate() {
        this.area = Double.NaN;
        this.horizontalOffset = Double.NaN;
        this.verticalOffset = Double.NaN;
        this.tagId = -1;
        this.tagPose = null;
        this.valid = false;
    }

    /**
     * Sets the properties of a valid estimate
     */
    public void set(double area,
                    double offsetX,
                    double offsetY,
                    int tagId) {
        this.area = area;
        this.horizontalOffset = offsetX;
        this.verticalOffset = offsetY;
        this.tagId = tagId;
        this.tagPose = Field.getTagPose(tagId);
        this.valid = true;
    }

    /**
     * @return do we actually have valid target information?
     */
    public boolean isValid() {
        return valid;
    }

    /**
     * @return the target area in the frame (roughly this is TA);
     * {@link Double#NaN} if there is no valid target
     */
    public double getArea() {
        return area;
    }

    /**
     * @return the horizontal offset of the frame (roughly this is TX);
     * {@link Double#NaN} if there is no valid target
     */
    public double getHorizontalOffset() {
        return horizontalOffset;
    }

    /**
     * @return the vertical offset of the frame (roughly this is TY);
     * {@link Double#NaN} if there is no valid target
     */
    public double getVerticalOffset() {
        return verticalOffset;
    }

    /**
     * @return the ID of the current in-view AprilTag; -1 if there is no
     * valid target
     */
    public int getTagId() {
        return tagId;
    }

    /**
     * @return the pose of the in-view AprilTag; null if there is no
     * valid target
     */
    public Pose2d getTagPose() {
        return tagPose;
    }
}

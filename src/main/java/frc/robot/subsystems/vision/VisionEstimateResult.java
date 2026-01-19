package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

/**
 * Result of a vision pose estimate attempt.
 * <p>
 * Contains the raw estimate from Limelight along with calculated
 * standard deviations and acceptance status.
 */
public class VisionEstimateResult {

    /** The raw pose estimate from Limelight (may be null if rejected early) */
    public final PoseEstimate estimate;

    /** Calculated standard deviation for X and Y (meters) */
    public final double stdDevXY;

    /** Whether this estimate was accepted and applied to the pose estimator */
    public final boolean accepted;

    /** Reason the estimate was rejected (empty string if accepted) */
    public final String rejectionReason;

    /** Whether this result came from MegaTag2 (false = Classic/MegaTag1) */
    public final boolean isMegaTag2;

    /**
     * Creates an accepted result.
     */
    public VisionEstimateResult(PoseEstimate estimate, double stdDevXY, boolean isMegaTag2) {
        this.estimate = estimate;
        this.stdDevXY = stdDevXY;
        this.accepted = true;
        this.rejectionReason = "";
        this.isMegaTag2 = isMegaTag2;
    }

    /**
     * Creates a rejected result.
     */
    public VisionEstimateResult(PoseEstimate estimate, String rejectionReason, boolean isMegaTag2) {
        this.estimate = estimate;
        this.stdDevXY = Double.NaN;
        this.accepted = false;
        this.rejectionReason = rejectionReason;
        this.isMegaTag2 = isMegaTag2;
    }

    /**
     * Creates a rejected result with no estimate.
     */
    public VisionEstimateResult(String rejectionReason) {
        this.estimate = null;
        this.stdDevXY = Double.NaN;
        this.accepted = false;
        this.rejectionReason = rejectionReason;
        this.isMegaTag2 = false;
    }

    /**
     * @return the estimated pose, or null if no estimate
     */
    public Pose2d getPose() {
        return estimate != null ? estimate.pose : null;
    }

    /**
     * @return the number of tags seen, or 0 if no estimate
     */
    public int getTagCount() {
        return estimate != null ? estimate.tagCount : 0;
    }

    /**
     * @return the average distance to tags in meters, or 0 if no estimate
     */
    public double getAvgTagDist() {
        return estimate != null ? estimate.avgTagDist : 0.0;
    }

    /**
     * @return the timestamp of the estimate, or 0 if no estimate
     */
    public double getTimestamp() {
        return estimate != null ? estimate.timestampSeconds : 0.0;
    }

    @Override
    public String toString() {
        if (accepted) {
            return String.format("VisionEstimateResult[accepted, tags=%d, stdDev=%.3f, mt2=%b]",
                    getTagCount(), stdDevXY, isMegaTag2);
        } else {
            return String.format("VisionEstimateResult[rejected: %s]", rejectionReason);
        }
    }
}

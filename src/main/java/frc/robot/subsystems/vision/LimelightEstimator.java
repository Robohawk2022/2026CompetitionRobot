package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.util.Util;

import static frc.robot.Config.Limelight.*;

/**
 * Helper methods for using Limelight pose estimation with swerve drive.
 * <p>
 * Supports two estimation strategies:
 * <ul>
 *   <li>Classic (MegaTag1): Standard AprilTag pose estimation</li>
 *   <li>MegaTag2: Uses robot orientation for improved accuracy</li>
 * </ul>
 * <p>
 * Features:
 * <ul>
 *   <li>Dynamic standard deviation based on tag count and distance</li>
 *   <li>Field boundary validation</li>
 *   <li>Pose jump detection</li>
 *   <li>Configurable filtering thresholds</li>
 * </ul>
 *
 * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">Limelight Swerve Tutorial</a>
 */
public class LimelightEstimator {

    /** Strategy constants */
    public static final int STRATEGY_CLASSIC = 0;
    public static final int STRATEGY_MEGATAG2 = 1;
    public static final int STRATEGY_BOTH = 2;

    /** Current active Point of Interest for tag filtering */
    private static TagPOI activePOI = TagPOI.ALL;

    /** Valid tag IDs based on active POI (null = no filtering) */
    private static int[] validTagIds = null;

    /** Last accepted pose for jump detection */
    private static Pose2d lastAcceptedPose = null;

    /**
     * Sets the active Point of Interest for tag filtering.
     * <p>
     * Only tags at the specified POI will be used for pose estimation.
     * This can improve accuracy when you know which part of the field you're near.
     *
     * @param poi the Point of Interest to filter by
     */
    public static void setActivePOI(TagPOI poi) {
        activePOI = poi;
        validTagIds = switch (poi) {
            // Game element based
            case TOWER -> TOWER_TAG_IDS;
            case OUTPOST -> OUTPOST_TAG_IDS;
            case HUB -> HUB_TAG_IDS;
            // Position based
            case LEFT_SIDE -> LEFT_SIDE_TAG_IDS;
            case RIGHT_SIDE -> RIGHT_SIDE_TAG_IDS;
            case CENTER -> CENTER_TAG_IDS;
            case ALLIANCE_WALL -> ALLIANCE_WALL_TAG_IDS;
            // No filtering
            case ALL -> null;
        };

        // Apply filter to Limelight
        if (validTagIds != null) {
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validTagIds);
        }

        SmartDashboard.putString("Vision/ActivePOI", poi.name());
    }

    /**
     * Gets the current active Point of Interest.
     *
     * @return the active POI
     */
    public static TagPOI getActivePOI() {
        return activePOI;
    }

    /**
     * Resets the last accepted pose. Call this when resetting the robot pose.
     */
    public static void resetLastPose() {
        lastAcceptedPose = null;
    }

    /**
     * Main entry point for vision pose estimation.
     * <p>
     * This method selects the appropriate strategy based on configuration,
     * validates the estimate, calculates standard deviations, and updates
     * the pose estimator if the estimate passes all checks.
     *
     * @param estimator   the pose estimator to update
     * @param yawDegrees  the current robot yaw in degrees (for MegaTag2)
     * @param yawRateDps  the current yaw rate in degrees per second
     * @param currentPose the current estimated pose (for jump detection)
     * @return the result of the vision estimate attempt
     */
    public static VisionEstimateResult updateEstimate(
            SwerveDrivePoseEstimator estimator,
            double yawDegrees,
            double yawRateDps,
            Pose2d currentPose) {

        // check master enable
        if (!visionEnabled.getAsBoolean()) {
            publishTelemetry(null, false, "disabled");
            return new VisionEstimateResult("vision disabled");
        }

        int strat = (int) strategy.getAsDouble();
        VisionEstimateResult result;

        if (strat == STRATEGY_CLASSIC) {
            result = processClassicEstimate(estimator, currentPose);
        } else if (strat == STRATEGY_MEGATAG2) {
            result = processMegaTag2Estimate(estimator, yawDegrees, yawRateDps, currentPose);
        } else {
            // STRATEGY_BOTH: Try MegaTag2 first, fall back to classic
            result = processMegaTag2Estimate(estimator, yawDegrees, yawRateDps, currentPose);
            if (!result.accepted) {
                result = processClassicEstimate(estimator, currentPose);
            }
        }

        // update last accepted pose
        if (result.accepted && result.getPose() != null) {
            lastAcceptedPose = result.getPose();
        }

        // publish telemetry
        publishTelemetry(result, result.accepted, result.rejectionReason);

        return result;
    }

    /**
     * Processes a Classic (MegaTag1) pose estimate.
     */
    private static VisionEstimateResult processClassicEstimate(
            SwerveDrivePoseEstimator estimator,
            Pose2d currentPose) {

        // use alliance-aware pose estimate method
        PoseEstimate estimate = Util.isRedAlliance()
                ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // basic validity check
        String basicReject = checkBasicValidity(estimate, false);
        if (basicReject != null) {
            return new VisionEstimateResult(estimate, basicReject, false);
        }

        // single-tag filtering
        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            double ambiguity = estimate.rawFiducials[0].ambiguity;
            double distToCamera = estimate.rawFiducials[0].distToCamera;

            if (ambiguity > maxAmbiguity.getAsDouble()) {
                return new VisionEstimateResult(estimate,
                        String.format("ambiguity %.2f > %.2f", ambiguity, maxAmbiguity.getAsDouble()), false);
            }
            if (distToCamera > maxDistToCamera.getAsDouble()) {
                return new VisionEstimateResult(estimate,
                        String.format("dist %.2fm > %.2fm", distToCamera, maxDistToCamera.getAsDouble()), false);
            }
        }

        // field boundary check
        String boundaryReject = checkFieldBoundary(estimate.pose);
        if (boundaryReject != null) {
            return new VisionEstimateResult(estimate, boundaryReject, false);
        }

        // pose jump check
        String jumpReject = checkPoseJump(estimate.pose, currentPose);
        if (jumpReject != null) {
            return new VisionEstimateResult(estimate, jumpReject, false);
        }

        // calculate dynamic standard deviation
        double stdDevXY = calculateStdDev(estimate.tagCount, estimate.avgTagDist);

        // apply to pose estimator
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevXY, stdDevXY, 9999999));
        estimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);

        return new VisionEstimateResult(estimate, stdDevXY, false);
    }

    /**
     * Processes a MegaTag2 pose estimate.
     */
    private static VisionEstimateResult processMegaTag2Estimate(
            SwerveDrivePoseEstimator estimator,
            double yawDegrees,
            double yawRateDps,
            Pose2d currentPose) {

        // provide robot orientation for MegaTag2
        LimelightHelpers.SetRobotOrientation(
                limelightName,
                yawDegrees,
                0.0, 0.0,  // roll rate, pitch rate
                0.0, 0.0, 0.0);  // additional parameters

        // reject if rotating too fast
        if (Math.abs(yawRateDps) > maxYawRateDps.getAsDouble()) {
            return new VisionEstimateResult(String.format("yaw rate %.1f > %.1f dps",
                    Math.abs(yawRateDps), maxYawRateDps.getAsDouble()));
        }

        // use alliance-aware MegaTag2 method
        PoseEstimate estimate = Util.isRedAlliance()
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // basic validity check
        String basicReject = checkBasicValidity(estimate, true);
        if (basicReject != null) {
            return new VisionEstimateResult(estimate, basicReject, true);
        }

        // tag area check (specific to MegaTag2)
        if (estimate.avgTagArea < minTagArea.getAsDouble()) {
            return new VisionEstimateResult(estimate,
                    String.format("tag area %.3f < %.3f", estimate.avgTagArea, minTagArea.getAsDouble()), true);
        }

        // field boundary check
        String boundaryReject = checkFieldBoundary(estimate.pose);
        if (boundaryReject != null) {
            return new VisionEstimateResult(estimate, boundaryReject, true);
        }

        // pose jump check
        String jumpReject = checkPoseJump(estimate.pose, currentPose);
        if (jumpReject != null) {
            return new VisionEstimateResult(estimate, jumpReject, true);
        }

        // calculate dynamic standard deviation
        double stdDevXY = calculateStdDev(estimate.tagCount, estimate.avgTagDist);

        // apply to pose estimator
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevXY, stdDevXY, 9999999));
        estimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);

        return new VisionEstimateResult(estimate, stdDevXY, true);
    }

    /**
     * Checks basic validity of a pose estimate.
     *
     * @param estimate   the estimate to check
     * @param isMegaTag2 whether this is a MegaTag2 estimate
     * @return rejection reason, or null if valid
     */
    private static String checkBasicValidity(PoseEstimate estimate, boolean isMegaTag2) {
        if (estimate == null) {
            return "null estimate";
        }
        if (estimate.tagCount == 0) {
            return "no tags";
        }
        return null;
    }

    /**
     * Checks if a pose is within field boundaries.
     *
     * @param pose the pose to check
     * @return rejection reason, or null if valid
     */
    private static String checkFieldBoundary(Pose2d pose) {
        double margin = FIELD_BOUNDARY_MARGIN;
        double x = pose.getX();
        double y = pose.getY();

        if (x < -margin || x > FIELD_LENGTH_METERS + margin) {
            return String.format("x=%.2fm out of bounds", x);
        }
        if (y < -margin || y > FIELD_WIDTH_METERS + margin) {
            return String.format("y=%.2fm out of bounds", y);
        }
        return null;
    }

    /**
     * Checks if the pose jump from current estimate is too large.
     *
     * @param visionPose  the new vision pose
     * @param currentPose the current estimated pose
     * @return rejection reason, or null if valid
     */
    private static String checkPoseJump(Pose2d visionPose, Pose2d currentPose) {
        // skip check if we don't have a reference pose
        if (lastAcceptedPose == null && currentPose == null) {
            return null;
        }

        // use last accepted pose if available, otherwise current pose
        Pose2d reference = lastAcceptedPose != null ? lastAcceptedPose : currentPose;
        if (reference == null) {
            return null;
        }

        double distance = visionPose.getTranslation().getDistance(reference.getTranslation());
        double maxJump = maxPoseJump.getAsDouble();

        if (distance > maxJump) {
            return String.format("pose jump %.2fm > %.2fm", distance, maxJump);
        }
        return null;
    }

    /**
     * Calculates dynamic standard deviation using ProjectBucephalus formula.
     * <p>
     * Formula: stdDev = baseline * (avgDist^2 / tagCount)
     * <ul>
     *   <li>Quadratic distance penalty (further = much less trust)</li>
     *   <li>Linear tag count benefit (more tags = more trust)</li>
     * </ul>
     *
     * @param tagCount number of tags seen
     * @param avgDist  average distance to tags in meters
     * @return calculated standard deviation in meters
     */
    private static double calculateStdDev(int tagCount, double avgDist) {
        double baseline = linearStdDevBaseline.getAsDouble();
        double stdDevFactor = Math.pow(avgDist, 2.0) / tagCount;
        return baseline * stdDevFactor;
    }

    /**
     * Publishes telemetry to SmartDashboard.
     */
    private static void publishTelemetry(VisionEstimateResult result, boolean valid, String status) {
        SmartDashboard.putBoolean("Vision/Valid", valid);
        SmartDashboard.putString("Vision/Status", status);

        if (result != null && result.estimate != null) {
            SmartDashboard.putNumber("Vision/TagCount", result.getTagCount());
            SmartDashboard.putNumber("Vision/AvgTagDist", result.getAvgTagDist());
            SmartDashboard.putNumber("Vision/StdDevXY", result.accepted ? result.stdDevXY : -1);
            SmartDashboard.putBoolean("Vision/IsMegaTag2", result.isMegaTag2);

            if (result.getPose() != null) {
                SmartDashboard.putNumber("Vision/PoseX", result.getPose().getX());
                SmartDashboard.putNumber("Vision/PoseY", result.getPose().getY());
                SmartDashboard.putNumber("Vision/PoseRot", result.getPose().getRotation().getDegrees());
            }
        } else {
            SmartDashboard.putNumber("Vision/TagCount", 0);
            SmartDashboard.putNumber("Vision/AvgTagDist", 0);
            SmartDashboard.putNumber("Vision/StdDevXY", -1);
        }

        SmartDashboard.putString("Vision/RejectionReason", valid ? "" : status);
    }

    //=========================================================================
    // Legacy methods for backwards compatibility
    //=========================================================================

    /**
     * Updates the pose estimator using classic (MegaTag1) pose estimation.
     * <p>
     * @deprecated Use {@link #updateEstimate(SwerveDrivePoseEstimator, double, double, Pose2d)} instead.
     */
    @Deprecated
    public static PoseEstimate updateEstimateClassic(
            String limelightName,
            SwerveDrivePoseEstimator estimator) {

        // use alliance-aware pose estimate method
        PoseEstimate estimate = Util.isRedAlliance()
                ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (estimate == null || estimate.tagCount == 0) {
            SmartDashboard.putBoolean("Vision/ClassicValid", false);
            return null;
        }

        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            if (estimate.rawFiducials[0].ambiguity > maxAmbiguity.getAsDouble()) {
                SmartDashboard.putBoolean("Vision/ClassicValid", false);
                return null;
            }
            if (estimate.rawFiducials[0].distToCamera > maxDistToCamera.getAsDouble()) {
                SmartDashboard.putBoolean("Vision/ClassicValid", false);
                return null;
            }
        }

        SmartDashboard.putBoolean("Vision/ClassicValid", true);
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
        estimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);

        return estimate;
    }

    /**
     * Updates the pose estimator using MegaTag2 pose estimation.
     * <p>
     * @deprecated Use {@link #updateEstimate(SwerveDrivePoseEstimator, double, double, Pose2d)} instead.
     */
    @Deprecated
    public static PoseEstimate updateEstimateMegaTag2(
            String limelightName,
            SwerveDrivePoseEstimator estimator,
            double yawDegrees,
            double yawRateDps) {

        LimelightHelpers.SetRobotOrientation(
                limelightName,
                yawDegrees,
                0.0, 0.0,
                0.0, 0.0, 0.0);

        if (Math.abs(yawRateDps) > maxYawRateDps.getAsDouble()) {
            SmartDashboard.putBoolean("Vision/MT2Valid", false);
            return null;
        }

        // use alliance-aware MegaTag2 method
        PoseEstimate estimate = Util.isRedAlliance()
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (estimate == null || estimate.tagCount == 0) {
            SmartDashboard.putBoolean("Vision/MT2Valid", false);
            return null;
        }

        SmartDashboard.putNumber("Vision/TagArea", estimate.avgTagArea);
        if (estimate.avgTagArea < minTagArea.getAsDouble()) {
            SmartDashboard.putBoolean("Vision/MT2Valid", false);
            return null;
        }

        SmartDashboard.putBoolean("Vision/MT2Valid", true);
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        estimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);

        return estimate;
    }
}

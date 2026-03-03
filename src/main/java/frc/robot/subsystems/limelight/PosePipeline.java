package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.limelight.LimelightHelpers.RawFiducial;
import frc.robot.util.Field;
import frc.robot.util.Field.HubSide;
import frc.robot.util.Util;

import java.util.Objects;

import static frc.robot.Config.Limelight.closeRange;

/**
 * Wraps a Limelight algorithm in a "pipeline" that will grab the most
 * recent robot pose estimate and determine whether it's high-enough
 * confidence that it can be used.
 */
public class PosePipeline {

    /**
     * When we perform fused estimation, we need to know how confident we are
     * in an estimate; MegaTag2 has different levels but MegaTag1 does not
     */
    public static final Vector<N3> CONF_LO = VecBuilder.fill(2.0, 2.0, 9999999.0);
    public static final Vector<N3> CONF_MED = VecBuilder.fill(0.9, 0.9, 9999999.0);
    public static final Vector<N3> CONF_HI = VecBuilder.fill(0.5, 0.5, 9999999.0);
    public static final Vector<N3> CONF_MT1 = VecBuilder.fill(0.5, 0.5, 9999999.0);

    /**
     * Determines which of our two algorithms a pipeline is going to use
     */
    public enum Algorithm {

        MegaTag1,
        MegaTag2;

        public PoseEstimate getEstimate(String limelightName) {
            return switch (this) {
                case MegaTag1 -> LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                case MegaTag2 -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            };
        }
    }

    final String limelightName;
    final Algorithm algorithm;
    final String estimatePoseKey;
    final String tagPoseKey;

    // counters for the various different results of the pipeline
    long noEstimate;
    long outsideField;
    long tagObscured;
    long ambiguous;
    long tagTooFar;
    long lowConfidenceCount;
    long mediumConfidenceCount;
    long highConfidenceCount;

    // state of the current estimate; updated each cycle
    PoseEstimate latestEstimate;
    Vector<N3> latestConfidence;

    /**
     * Creates a {@link PosePipeline}
     * @param limelightName the name of the limelight in NetworkTables
     * @param algorithm which algorithm to use
     */
    public PosePipeline(String limelightName, Algorithm algorithm) {
        this.limelightName = Objects.requireNonNull(limelightName);
        this.algorithm = Objects.requireNonNull(algorithm);
        this.latestEstimate = null;
        this.latestConfidence = null;
        this.estimatePoseKey = algorithm.name()+"-RobotPose";
        this.tagPoseKey = algorithm.name()+"-TagPose";
    }

    public void addToBuilder(SendableBuilder builder) {

        // high-level status indicators
        builder.addBooleanProperty(algorithm.name()+"/HasEstimate?", () -> latestEstimate != null, null);

        // error counters (some are unique to MegaTag1)
        builder.addIntegerProperty(algorithm.name()+"/ErrNoEstimate", () -> noEstimate, null);
        builder.addIntegerProperty(algorithm.name()+"/ErrOutsideField", () -> outsideField, null);
        builder.addIntegerProperty(algorithm.name()+"/ErrTagObscured", () -> tagObscured, null);
        if (algorithm == Algorithm.MegaTag1) {
            builder.addIntegerProperty(algorithm.name()+"/ErrAmbiguous", () -> ambiguous, null);
            builder.addIntegerProperty(algorithm.name()+"/ErrTagTooFar", () -> tagTooFar, null);
        }

        // counter for usable estimates by confidence level
        if (algorithm == Algorithm.MegaTag1) {
            builder.addIntegerProperty(algorithm.name()+"/Ok", () -> lowConfidenceCount, null);
        } else {
            builder.addIntegerProperty(algorithm.name()+"/Ok-1-Low", () -> lowConfidenceCount, null);
            builder.addIntegerProperty(algorithm.name()+"/Ok-2-Med", () -> mediumConfidenceCount, null);
            builder.addIntegerProperty(algorithm.name()+"/Ok-3-High", () -> highConfidenceCount, null);
        }
    }

    /**
     * @return the latest calculated pose estimate (null if there isn't one)
     */
    public PoseEstimate getLatestEstimate() {
        return latestEstimate;
    }

    /**
     * @return the pose of the tag used for the most recent estimate (null
     * if there isn't one)
     */
    public Pose2d getTagPose() {
        return latestEstimate == null
                ? null
                : Field.getTagPose(latestEstimate.rawFiducials[0].id);
    }

    /**
     * @return the latest calculated confidence (null if there is no estimate,
     * or it shouldn't be used for fused estimation)
     */
    public Vector<N3> getLatestConfidence() {
        return latestConfidence;
    }

//region Update ----------------------------------------------------------------

    public void update(Pose2d currentPose) {

        latestEstimate = null;
        latestConfidence = null;

        PoseEstimate estimate = algorithm.getEstimate(limelightName);

        // if there's no estimate, we should ignore it (but we are not "wonky")
        if (notEvenAThing(estimate)) {
            noEstimate++;
            return;
        }

        // if the estimated pose is outside the field, we should ignore it
        // (but we are not "wonky")
        if (Field.isOutsideField(estimate.pose)) {
            outsideField++;
            return;
        }

        // special checks for MegaTag1 algorithm
        // we're just using hardcoded vendor recommended thresholds
        // see https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
        if (algorithm == Algorithm.MegaTag1) {

            RawFiducial fiducial = estimate.rawFiducials[0];

            if (fiducial.ambiguity > 0.7) {
                ambiguous++;
                return;
            }

            if (fiducial.distToCamera > 3.0) {
                tagTooFar++;
                return;
            }
        }

        // publish the pose for debugging
        Util.publishPose(estimatePoseKey, estimate.pose);
        Util.publishPose(tagPoseKey, Field.getTagPose(estimate.rawFiducials[0].id));

        // if the hub would obscure the tag from view, this is obviously
        // not a good estimate and should ignore it (this is an error condition
        // we've noticed in practice)
        if (tagNotVisibleFromPose(estimate)) {
            tagObscured++;
            return;
        }

        // at this point, we have an estimate and can determine confidence
        latestEstimate = estimate;
        latestConfidence = determineConfidence(estimate);
    }

//endregion

//region Helpers ---------------------------------------------------------------

    /**
     * @return return if the supplied estimate is null, or lacks enough
     * information to be used for the robot's pose
     */
    private boolean notEvenAThing(PoseEstimate estimate) {
        return estimate == null
                || estimate.pose == null
                || estimate.rawFiducials == null
                || estimate.rawFiducials.length == 0;
    }

    /**
     * @return true if the estimate is based on a hub tag which would be
     * obscured from view by the robot if it were at the estimated pose
     * (this is an error condition we've seen with LL estimation before)
     */
    private boolean tagNotVisibleFromPose(PoseEstimate estimate) {

        HubSide hubSide = Field.getHubSide(estimate.rawFiducials[0].id);

        // if the estimate isn't based on a hub tag, we don't have an opinion
        // on visibility
        if (hubSide == null) {
            return false;
        }

        // in each case, we expect the estimated robot pose to be in the same
        // rough "zone" as the tag that was used to generate it
        return switch (hubSide) {
            case NORTH -> Field.isNorthSide(estimate.pose);
            case SOUTH -> Field.isSouthSide(estimate.pose);
            case NEUTRAL -> Field.isNeutralZone(estimate.pose);
            case ALLIANCE -> Field.isAllianceZone(estimate.pose);
        };
    }

    /**
     * @return our confidence level in the supplied estimate; this will
     * depend on which algorithm we're using and other factors
     */
    private Vector<N3> determineConfidence(PoseEstimate estimate) {

        // MegaTag1 only has one confidence level
        if (algorithm == Algorithm.MegaTag1) {
            lowConfidenceCount++;
            return CONF_MT1;
        }

        boolean multiTag = estimate.rawFiducials.length > 1;
        boolean nearDistance = Units.metersToFeet(estimate.avgTagDist) < closeRange.getAsDouble();

        // close-up multi-tag estimates are the bomb!
        if (multiTag && nearDistance) {
            highConfidenceCount++;
            return CONF_HI;
        }

        // multi-tag and far off, or close by, are ok
        else if (multiTag || nearDistance) {
            mediumConfidenceCount++;
            return CONF_MED;
        }

        // all others are low-confidence
        else {
            lowConfidenceCount++;
            return CONF_LO;
        }
    }

//endregion

}

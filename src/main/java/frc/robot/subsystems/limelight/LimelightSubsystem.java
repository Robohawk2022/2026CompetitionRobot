package frc.robot.subsystems.limelight;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config.Limelight;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.Util;

import java.util.Objects;

import static frc.robot.Config.Limelight.closeRange;
import static frc.robot.Config.Limelight.enabled;
import static frc.robot.Config.Limelight.limelightName;
import static frc.robot.Config.Limelight.maxPoseJumpFeet;
import static frc.robot.Config.Limelight.maxYawRate;

/**
 * Subsystem that uses the Limelight to:
 * <ul>
 *
 *     <li>Generate pose estimates during <code>periodic</code> and, if
 *     they pass relevant quality checks, feed them to a consumer (e.g. the
 *     swerve drive)</li>
 *
 *     <li>Use Limelight's targeting pipeline to report on the physical
 *     position of a single target as a {@link LimelightTarget} (with the
 *     option to restrict to a single AprilTag of interest)</li>
 *
 * </ul>
 */
public class LimelightSubsystem extends SubsystemBase {

    /** Enable verbose logging for debugging */
    static final boolean verboseLogging = true;

    final SwerveSubsystem swerve;
    final LimelightTarget latestTarget;
    final LimelightResults results;
    double lastTime;
    double lastYaw;
    double lastYawRate;
    boolean spinningTooFast;
    Pose2d latestEstimate;
    Pose2d latestTagPose;
    Pose2d latestTargetPose;
    long restrictedTag;
    long noEstimate;
    long noTags;
    long poseOutsideField;
    long poseJumpTooBig;
    long lowConfidenceCount;
    long highConfidenceCount;
    long mediumConfidenceCount;
    boolean forceResetPose;
    boolean estimateSuccessful;
    boolean resetPose;

    /**
     * Creates a {@link LimelightSubsystem}
     * @param swerve the swerve drive
     * @throws IllegalArgumentException if required parameters are null
     */
    public LimelightSubsystem(SwerveSubsystem swerve) {

        this.swerve = Objects.requireNonNull(swerve);
        this.latestTarget = new LimelightTarget();
        this.results = new LimelightResults();
        this.latestEstimate = null;
        this.latestTagPose = null;
        this.forceResetPose = false;
        this.estimateSuccessful = false;

        SmartDashboard.putData("LimelightSubsystem", builder -> {
            builder.addIntegerProperty("RestrictedTag", () -> restrictedTag, null);
            builder.addBooleanProperty("TooFast?", () -> spinningTooFast, null);
            builder.addBooleanProperty("HasEstimate?", () -> estimateSuccessful, null);
            builder.addBooleanProperty("HasTarget?", latestTarget::isValid, null);
            builder.addBooleanProperty("ResetPose?", () -> forceResetPose, val -> forceResetPose = val);
            if (verboseLogging) {
                latestTarget.addToDashboard(builder);
                results.addToDashboard(builder);
            }
        });
    }

    /**
     * Indicates that only one specific AprilTag should be used for the
     * {@link LimelightTarget}; all other tags should be ignored
     * @param id the tag of interest
     */
    public void setRestrictedTag(int id) {
        restrictedTag = id;
        LimelightHelpers.setPriorityTagID(limelightName, id);
    }

    /**
     * Clears the "target tag" designation
     * TODO is this the right way to do this?
     */
    public void clearRestrictedTag() {
        restrictedTag = -1L;
        LimelightHelpers.setPriorityTagID(limelightName, -1);
    }

//region Periodic --------------------------------------------------------------

    @Override
    public void periodic() {

        // if we're enabled, update all of our information
        if (enabled.getAsBoolean()) {
            Pose2d currentPose = swerve.getPose();
            updateTargeting();
            updateYawRate(currentPose);
            updatePoseEstimate(currentPose);
        } else {
            latestEstimate = null;
            latestTarget.invalidate();
        }

        // publish poses
        Util.publishPose("LimelightTarget", latestTarget.getTagPose());
        Util.publishPose("limelightEstimate", latestEstimate);
        Util.publishPose("LimelightEstimateTag", latestTagPose);
    }

//endregion

//region Targeting -------------------------------------------------------------

    /**
     * Updates targeting information if it's available
     */
    private void updateTargeting() {

        // check if we even have a target acquired
        if (!LimelightHelpers.getTV(limelightName)) {
            latestTarget.invalidate();
            return;
        }

        // we want the horizontal offset to be negative when the tag is to the
        // left (so positive motion will decrease it). this is the opposite of
        // how the camera reports it, so we will negate it before returning
        double ho = -LimelightHelpers.getTX(limelightName);

        // other data can be consumed as provided
        double vo = LimelightHelpers.getTY(limelightName);
        double ta = LimelightHelpers.getTA(limelightName);
        int tid = (int) LimelightHelpers.getFiducialID(limelightName);

        latestTarget.set(ta, ho, vo, tid);
    }

//endregion

//region Yaw rate --------------------------------------------------------------

    /**
     * Grabs the current yaw of the robot and compares it to the yaw the
     * last time the method was called to compute a yaw rate. If we can
     * calculate one, we'll update the Limelight since it wants to know
     * how fast we're spinning.
     */
    private void updateYawRate(Pose2d currentPose) {

        double currentYaw = currentPose.getRotation().getDegrees();
        double currentTime = Timer.getFPGATimestamp();

        // the first time through we won't have a "last" yaw, so we can't
        // really perform the calculation; this will only trigger the
        // second and subsequent times through
        if (Double.isFinite(lastYaw)) {
            lastYawRate = (currentYaw - lastYaw) / (currentTime - lastTime);
            LimelightHelpers.SetRobotOrientation(
                    limelightName,
                    lastYaw,
                    lastYawRate, 0.0, 0.0, 0.0, 0.0);
        }

        // if we couldn't calculate a rate, or we did and it's too big,
        // we are spinning too fast for estimates to work
        spinningTooFast = Double.isNaN(lastYawRate)
                || lastYawRate > maxYawRate.getAsDouble();

        lastYaw = currentYaw;
        lastTime = currentTime;
    }

//endregion

//region Pose estimate ---------------------------------------------------------

    /**
     * Calculates a pose estimate using the LL MegaTag2 algorithm, passes it
     * through some quality gates, and possibly supplies it to the drive.
     */
    private void updatePoseEstimate(Pose2d currentPose) {

        latestEstimate = null;
        latestTagPose = null;
        estimateSuccessful = false;

        // if we're spinning too fast, there's nothing to do
        if (spinningTooFast) {
            return;
        }

        // grab the estimate; if there isn't one, we're done
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null || estimate.pose == null) {
            results.noEstimate();
            return;
        }

        // if the estimate somehow isn't based on fiducials, we won't use it
        // (this would be really weird and is more of a sanity check)
        if (estimate.rawFiducials == null || estimate.rawFiducials.length < 1) {
            results.noTags();
            return;
        }

        latestEstimate = estimate.pose;

        // if the pose is outside the field, there's no use dealing with it
        if (Field.isOutsideField(estimate.pose)) {
            results.poseOutsideField();
            return;
        }

        // if we're resetting the robot's pose, we will do that now with the
        // estimate we have
        if (forceResetPose) {
            swerve.resetPose(estimate.pose);
            forceResetPose = false;
        }

        // if the pose is too far from the current pose, we don't submit it ...
        // UNLESS a pose reset has been requested
        // we'll remember where the latest estimate and latest tag pose are
        latestEstimate = estimate.pose;
        latestTagPose = Field.getTagPose(estimate.rawFiducials[0].id);

        // if we're forced a pose reset, we'll do it here (before we check
        // how far we are from the current pose)
        if (resetPose) {
            swerve.resetPose(latestEstimate);
            resetPose = false;
        }

        // if the pose is too far from the current pose, we don't submit it
        if (Util.feetBetween(currentPose, estimate.pose) > maxPoseJumpFeet.getAsDouble()) {
            results.poseJumpTooBig();
            return;
        }

        // let's figure out our confidence level
        boolean multiTag = estimate.rawFiducials.length > 1;
        boolean nearDistance = Units.metersToFeet(estimate.avgTagDist) < closeRange.getAsDouble();
        Vector<N3> confidence;

        // close-up multi-tag estimates are the bomb!
        if (multiTag && nearDistance) {
            results.highConfidence();
            confidence = Limelight.highConfidence;
        }

        // multi-tag and far off, or close by, are ok
        else if (multiTag || nearDistance) {
            results.mediumConfidence();
            confidence = Limelight.mediumConfidence;
        }

        // all others are low-confidence
        else {
            results.lowConfidence();
            confidence = Limelight.lowConfidence;
        }

        // we're good to go!
        swerve.submitVisionEstimate(estimate.pose, estimate.timestampSeconds, confidence);
        estimateSuccessful = true;
        latestTagPose = Field.getTagPose(estimate.rawFiducials[0].id);
    }

//endregion

//region Pose estimate ---------------------------------------------------------

    public Command resetPoseFromVisionCommand() {
        return runOnce(() -> forceResetPose = true);
    }

//endregion

}
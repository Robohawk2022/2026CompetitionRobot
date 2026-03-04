package frc.robot.subsystems.limelight;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.limelight.PosePipeline.Algorithm;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.Util;

import java.util.Objects;

import static frc.robot.Config.Limelight.enabled;
import static frc.robot.Config.Limelight.limelightName;
import static frc.robot.Config.Limelight.maxPoseJumpFeet;
import static frc.robot.Config.Limelight.maxYawRate;

/**
 * Subsystem that uses the Limelight to generate pose estimates. Notes:
 * <ul>
 *
 *     <li>We will use pose estimates from either the MegaTag1 or MegaTag2
 *     algorithms, but not both. If both are available, we'll take MegaTag2.
 *     </li>
 *
 *     <li>If the pose estimate is too far from the robot's current pose
 *     estimate, we will not use it for fused estimation. It can, however,
 *     be used to reset the robot's pose. This is super-helpful e.g. on
 *     practice fields.</li>
 *
 *     <li>If MegaTag2 supplies us with a good estimate at least once, and
 *     then starts providing estimates which are too far away from the
 *     robot's current pose, we will keep track of that and recommend a
 *     reset.</li>
 *
 * </ul>
 */
public class LimelightSubsystem extends SubsystemBase {

    /**
     * If MegaTag2 estimates are continuously too far from the robot's pose
     * for this amount of time, we will consider odometry "broken"
     */
    public static final double POSE_JUMP_TIMEOUT = 4.0;

    /**
     * Enable verbose logging for debugging
     */
    static final boolean verboseLogging = true;

    final SwerveSubsystem swerve;
    final Debouncer debouncer;
    PosePipeline megaTag1;
    PosePipeline megaTag2;
    String poseAlgorithm;
    double lastTime;
    double lastYaw;
    double lastYawRate;
    boolean spinningTooFast;
    boolean poseResetRecommended;
    boolean poseResetRequested;
    boolean fusedAtLeastOnce;

    /**
     * Creates a {@link LimelightSubsystem}
     * @param swerve the swerve drive
     * @throws IllegalArgumentException if required parameters are null
     */
    public LimelightSubsystem(SwerveSubsystem swerve) {

        this.swerve = Objects.requireNonNull(swerve);
        this.debouncer = new Debouncer(POSE_JUMP_TIMEOUT, DebounceType.kRising);
        this.megaTag1 = new PosePipeline(limelightName, Algorithm.MegaTag1);
        this.megaTag2 = new PosePipeline(limelightName, Algorithm.MegaTag2);
        this.lastTime = Double.NaN;
        this.lastYaw = Double.NaN;
        this.lastYawRate = Double.NaN;
        this.spinningTooFast = false;
        this.poseResetRequested = false;
        this.fusedAtLeastOnce = false;
        this.poseAlgorithm = "";

        SmartDashboard.putData("LimelightSubsystem", builder -> {
            builder.addBooleanProperty("TooFast?", () -> spinningTooFast, null);
            builder.addBooleanProperty("ResetRecommended?", () -> poseResetRecommended, null);
            builder.addBooleanProperty("ForceReset?", () -> poseResetRequested, val -> poseResetRequested = val);
            builder.addStringProperty("PoseAlgorithm", () -> poseAlgorithm, null);
            if (verboseLogging) {
                megaTag1.addToBuilder(builder);
                megaTag2.addToBuilder(builder);
            }
        });
    }

    public boolean isPoseResetRecommended() {
        return poseResetRecommended;
    }

    @Override
    public void periodic() {
        if (enabled.getAsBoolean()) {
            Pose2d currentPose = swerve.getPose();
            updateYawRate(currentPose);
            updatePoseEstimate(currentPose);
        }
    }

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

            // wrap the yaw difference to (-180, 180] so crossing the ±180°
            // boundary doesn't produce a huge spike (e.g. 179° → -179°)
            double yawDelta = Util.degreeModulus(currentYaw - lastYaw);

            lastYawRate = yawDelta / (currentTime - lastTime);
            LimelightHelpers.SetRobotOrientation(
                    limelightName,
                    currentYaw,
                    lastYawRate, 0.0, 0.0, 0.0, 0.0);
        }

        // if we couldn't calculate a rate, or we did and it's too big,
        // we are spinning too fast for estimates to work
        spinningTooFast = Double.isNaN(lastYawRate)
                || Math.abs(lastYawRate) > maxYawRate.getAsDouble();

        lastYaw = currentYaw;
        lastTime = currentTime;
    }

//endregion

//region Pose estimate ---------------------------------------------------------

    /**
     * Calculates a pose estimate. We will update both algorithms but only
     * supply one estimate to the swerve drive. We prefer MegaTag2 if it's
     * available.
     */
    private void updatePoseEstimate(Pose2d currentPose) {

        // if we're spinning too fast, we won't bother with this
        if (spinningTooFast) {
            return;
        }

        // update the two pipelines
        megaTag1.update(currentPose);
        megaTag2.update(currentPose);

        // publish poses
        if (verboseLogging) {
            Util.publishPose("MegaTag1-Robot", megaTag1.getRobotPose());
            Util.publishPose("MegaTag1-Tag", megaTag1.getTagPose());
            Util.publishPose("MegaTag2-Robot", megaTag2.getRobotPose());
            Util.publishPose("MegaTag2-Tag", megaTag2.getTagPose());
        }

        // determine whether we have an estimate we can use for fused
        // estimation (prefer MegaTag2 if available)
        if (attemptFusedEstimation(megaTag2, currentPose)) {
            poseAlgorithm = "MT2";
            fusedAtLeastOnce = true;
        } else if (attemptFusedEstimation(megaTag1, currentPose)) {
            poseAlgorithm = "MT1";
        } else {
            poseAlgorithm = "";
        }

        // if a pose reset was requested and we have a pose available,
        // let's do it (again, we prefer MegaTag2 if possible)
        if (poseResetRequested) {
            attemptPoseReset(megaTag2);
        }
        if (poseResetRequested) {
            attemptPoseReset(megaTag1);
        }
    }

    /**
     * Attempts to update fused pose estimation with the supplied pipeline
     * @return true if it works; false otherwise
     */
    private boolean attemptFusedEstimation(PosePipeline pipeline, Pose2d currentPose) {

        PoseEstimate estimate = pipeline.getLatestEstimate();
        if (estimate == null) {
            return false;
        }

        // if the pose estimate is too far from the robot's current pose, we
        // don't want to use it.
        boolean poseJumpTooFar = Util.feetBetween(currentPose, estimate.pose) > maxPoseJumpFeet.getAsDouble();

        // if we've had at least one usable pose estimate from MegaTag2, and now
        // we're getting bad ones, this is a sign that odometry has drifted in
        // a match. if this persists, we will recommend a pose reset.
        if (pipeline == megaTag2) {
            poseResetRecommended = debouncer.calculate(fusedAtLeastOnce && poseJumpTooFar);
        }

        if (poseJumpTooFar) {
            return false;
        }

        swerve.submitVisionEstimate(
                estimate.pose,
                estimate.timestampSeconds,
                pipeline.getLatestConfidence());
        return true;
    }

    /**
     * Uses the latest estimate on the supplied pipeline to forcibly reset
     * the swerve drive's pose
     */
    private void attemptPoseReset(PosePipeline pipeline) {
        PoseEstimate estimate = pipeline.getLatestEstimate();
        if (estimate != null) {
            swerve.resetPose(pipeline.getLatestEstimate().pose);
            poseResetRequested = false;
        }
    }

//endregion

//region Commands --------------------------------------------------------------

    public Command resetPoseFromVisionCommand() {
        return runOnce(() -> poseResetRequested = true);
    }

//endregion

}
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionEstimateResult;
import frc.robot.util.CircularBuffer;
import frc.robot.util.Util;

import java.util.HashMap;
import java.util.Map;

/**
 * Tracks and publishes odometry health diagnostics to the dashboard.
 * <p>
 * Monitors:
 * <ul>
 *   <li>Drift between odometry and fused pose (current, max, average)</li>
 *   <li>Vision acceptance rate and rejection reasons</li>
 *   <li>Odometry update frequency</li>
 *   <li>Vision measurement latency</li>
 * </ul>
 */
public class OdometryDiagnostics {

    /** Number of samples to keep for rolling averages */
    private static final int ROLLING_WINDOW_SIZE = 100;

    /** Dashboard key prefix */
    private static final String PREFIX = "Odom/";

    // drift tracking
    private final CircularBuffer<Double> driftHistory = new CircularBuffer<>(ROLLING_WINDOW_SIZE);
    private double currentDrift = 0.0;
    private double maxDrift = 0.0;

    // vision stats
    private int visionAccepted = 0;
    private int visionRejected = 0;
    private final Map<String, Integer> rejectionCounts = new HashMap<>();
    private double latestVisionLatency = 0.0;

    // odometry stats
    private double odometryFrequency = 0.0;
    private int lastUpdateCount = 0;
    private double lastFrequencyCalcTime = 0.0;

    /**
     * Creates a new OdometryDiagnostics instance.
     */
    public OdometryDiagnostics() {
        // initialize common rejection reasons
        rejectionCounts.put("NoTags", 0);
        rejectionCounts.put("Ambiguity", 0);
        rejectionCounts.put("TooFar", 0);
        rejectionCounts.put("OutOfBounds", 0);
        rejectionCounts.put("PoseJump", 0);
        rejectionCounts.put("HighYawRate", 0);
        rejectionCounts.put("SmallTags", 0);
        rejectionCounts.put("Disabled", 0);
        rejectionCounts.put("Other", 0);
    }

    /**
     * Updates diagnostics with the latest poses and vision result.
     *
     * @param odometryPose the raw odometry pose
     * @param fusedPose the fused/estimated pose (with vision corrections)
     * @param visionResult the latest vision result (may be null)
     * @param odomThread the odometry thread (may be null if not using HF odometry)
     */
    public void update(Pose2d odometryPose, Pose2d fusedPose,
                       VisionEstimateResult visionResult, OdometryThread odomThread) {

        // update drift tracking
        currentDrift = Util.metersBetween(odometryPose, fusedPose);
        driftHistory.add(currentDrift);
        maxDrift = Math.max(maxDrift, currentDrift);

        // update vision stats
        if (visionResult != null) {
            if (visionResult.accepted) {
                visionAccepted++;
                latestVisionLatency = calculateVisionLatency(visionResult);
            } else {
                visionRejected++;
                String reason = categorizeRejection(visionResult.rejectionReason);
                rejectionCounts.merge(reason, 1, Integer::sum);
            }
        }

        // update odometry frequency from thread
        if (odomThread != null) {
            updateFrequencyEstimate(odomThread);
        }
    }

    /**
     * Calculates the vision measurement latency.
     */
    private double calculateVisionLatency(VisionEstimateResult result) {
        if (result.estimate != null) {
            double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            return (now - result.estimate.timestampSeconds) * 1000.0; // convert to ms
        }
        return 0.0;
    }

    /**
     * Updates the odometry frequency estimate from the thread.
     */
    private void updateFrequencyEstimate(OdometryThread thread) {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        int currentCount = thread.getUpdateCount();

        if (lastFrequencyCalcTime > 0.0) {
            double elapsed = now - lastFrequencyCalcTime;
            if (elapsed >= 1.0) { // update every second
                int updates = currentCount - lastUpdateCount;
                odometryFrequency = updates / elapsed;
                lastUpdateCount = currentCount;
                lastFrequencyCalcTime = now;
            }
        } else {
            lastFrequencyCalcTime = now;
            lastUpdateCount = currentCount;
        }
    }

    /**
     * Categorizes a rejection reason into a known category.
     */
    private String categorizeRejection(String reason) {
        if (reason == null || reason.isEmpty()) {
            return "Other";
        }
        String lower = reason.toLowerCase();
        if (lower.contains("no tag") || lower.contains("notag")) {
            return "NoTags";
        }
        if (lower.contains("ambig")) {
            return "Ambiguity";
        }
        if (lower.contains("far") || lower.contains("dist")) {
            return "TooFar";
        }
        if (lower.contains("bound") || lower.contains("field")) {
            return "OutOfBounds";
        }
        if (lower.contains("jump")) {
            return "PoseJump";
        }
        if (lower.contains("yaw") || lower.contains("spin") || lower.contains("rotat")) {
            return "HighYawRate";
        }
        if (lower.contains("small") || lower.contains("area")) {
            return "SmallTags";
        }
        if (lower.contains("disable")) {
            return "Disabled";
        }
        return "Other";
    }

    /**
     * Publishes all diagnostics to SmartDashboard.
     */
    public void publishTelemetry() {
        // drift metrics
        SmartDashboard.putNumber(PREFIX + "DriftCurrent_m", currentDrift);
        SmartDashboard.putNumber(PREFIX + "DriftMax_m", maxDrift);
        SmartDashboard.putNumber(PREFIX + "DriftAvg_m", getDriftAverage());

        // vision metrics
        SmartDashboard.putNumber(PREFIX + "VisionAcceptRate_%", getVisionAcceptanceRate());
        SmartDashboard.putNumber(PREFIX + "VisionLatency_ms", latestVisionLatency);
        SmartDashboard.putNumber(PREFIX + "VisionAccepted", visionAccepted);
        SmartDashboard.putNumber(PREFIX + "VisionRejected", visionRejected);

        // odometry frequency
        SmartDashboard.putNumber(PREFIX + "OdomFrequency_Hz", odometryFrequency);

        // rejection counts
        for (Map.Entry<String, Integer> entry : rejectionCounts.entrySet()) {
            SmartDashboard.putNumber(PREFIX + "Reject/" + entry.getKey(), entry.getValue());
        }
    }

    /**
     * @return the rolling average drift in meters
     */
    public double getDriftAverage() {
        return driftHistory.average(d -> d);
    }

    /**
     * @return the maximum drift observed in meters
     */
    public double getMaxDrift() {
        return maxDrift;
    }

    /**
     * @return the current drift in meters
     */
    public double getCurrentDrift() {
        return currentDrift;
    }

    /**
     * @return the vision acceptance rate as a percentage (0-100)
     */
    public double getVisionAcceptanceRate() {
        int total = visionAccepted + visionRejected;
        if (total == 0) {
            return 0.0;
        }
        return (visionAccepted * 100.0) / total;
    }

    /**
     * @return the latest vision latency in milliseconds
     */
    public double getLatestVisionLatency() {
        return latestVisionLatency;
    }

    /**
     * @return the odometry update frequency in Hz
     */
    public double getOdometryFrequency() {
        return odometryFrequency;
    }

    /**
     * @return the number of accepted vision measurements
     */
    public int getVisionAcceptedCount() {
        return visionAccepted;
    }

    /**
     * @return the number of rejected vision measurements
     */
    public int getVisionRejectedCount() {
        return visionRejected;
    }

    /**
     * @return the rejection counts by reason
     */
    public Map<String, Integer> getRejectionCounts() {
        return new HashMap<>(rejectionCounts);
    }

    /**
     * Resets all diagnostics counters.
     */
    public void reset() {
        driftHistory.clear();
        currentDrift = 0.0;
        maxDrift = 0.0;
        visionAccepted = 0;
        visionRejected = 0;
        rejectionCounts.replaceAll((k, v) -> 0);
        latestVisionLatency = 0.0;
        odometryFrequency = 0.0;
        lastUpdateCount = 0;
        lastFrequencyCalcTime = 0.0;
    }
}

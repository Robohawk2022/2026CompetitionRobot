package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

import static frc.robot.Config.LimelightSim.*;
import static frc.robot.Config.Limelight.limelightName;

/**
 * Simulates a Limelight camera for AprilTag detection.
 * <p>
 * This class simulates realistic Limelight behavior by:
 * <ul>
 *   <li>Checking tag visibility based on distance, FOV, and orientation</li>
 *   <li>Calculating tx/ty angles and tag area</li>
 *   <li>Adding configurable noise to pose estimates</li>
 *   <li>Simulating frame drops and latency</li>
 *   <li>Publishing to NetworkTables in the exact format LimelightHelpers expects</li>
 * </ul>
 */
public class LimelightHardwareSim {

    /** Enable verbose logging */
    private static final boolean verboseLogging = false;

    /** Random number generator for noise simulation */
    private final Random random = new Random();

    /** Field layout containing all AprilTag positions */
    private final AprilTagFieldLayout fieldLayout;

    private final DoubleArrayPublisher botposePublisher;
    private final DoubleArrayPublisher botposeOrbPublisher;
    private final DoublePublisher tvPublisher;
    private final DoublePublisher txPublisher;
    private final DoublePublisher tyPublisher;
    private final DoublePublisher taPublisher;
    private final DoublePublisher tidPublisher;

    /** Tracking for frame timing */
    private double lastUpdateTime = 0;
    private int frameCount = 0;

    /**
     * Represents a detected AprilTag with its metrics.
     */
    private static class DetectedTag {
        final int id;
        final double distanceToCamera;
        final double distanceToRobot;
        final double tx;  // horizontal angle in degrees
        final double ty;  // vertical angle in degrees
        final double area; // area as percentage of image (0-100)
        final double ambiguity;
        final Pose3d tagPose;

        DetectedTag(int id, double distanceToCamera, double distanceToRobot,
                    double tx, double ty, double area, double ambiguity, Pose3d tagPose) {
            this.id = id;
            this.distanceToCamera = distanceToCamera;
            this.distanceToRobot = distanceToRobot;
            this.tx = tx;
            this.ty = ty;
            this.area = area;
            this.ambiguity = ambiguity;
            this.tagPose = tagPose;
        }
    }

    /**
     * Creates a new LimelightHardwareSim.
     */
    public LimelightHardwareSim() {
        this.fieldLayout = Util.getFieldLayout();

        // get NetworkTables table for the limelight
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);

        // create publishers
        botposePublisher = table.getDoubleArrayTopic("botpose_wpiblue").publish();
        botposeOrbPublisher = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish();
        tvPublisher = table.getDoubleTopic("tv").publish();
        txPublisher = table.getDoubleTopic("tx").publish();
        tyPublisher = table.getDoubleTopic("ty").publish();
        taPublisher = table.getDoubleTopic("ta").publish();
        tidPublisher = table.getDoubleTopic("tid").publish();

        Util.log("LimelightHardwareSim initialized with %d tags on field", fieldLayout.getTags().size());
    }

    /**
     * Updates the simulation based on the current robot pose.
     * <p>
     * Call this every periodic cycle before the LimelightEstimator processes data.
     *
     * @param robotPose the current robot pose (from odometry or ground truth)
     */
    public void update(Pose2d robotPose) {
        if (!enabled.getAsBoolean()) {
            publishNoTargets();
            return;
        }

        // simulate frame drops
        if (random.nextDouble() < frameDropProbability.getAsDouble()) {
            if (verboseLogging) {
                Util.log("LimelightHardwareSim: frame dropped");
            }
            // don't update NetworkTables on dropped frames
            return;
        }

        // calculate camera pose in 3D
        Pose3d cameraPose = calculateCameraPose(robotPose);

        // find all visible tags
        List<DetectedTag> visibleTags = findVisibleTags(cameraPose, robotPose);

        if (visibleTags.isEmpty()) {
            publishNoTargets();
        } else {
            // sort by distance (closest first for primary target)
            visibleTags.sort(Comparator.comparingDouble(t -> t.distanceToCamera));

            // add noise to the robot pose
            Pose2d noisyPose = addNoise(robotPose);

            // publish both classic and MegaTag2 poses
            publishClassicPose(noisyPose, visibleTags);
            publishMegaTag2Pose(noisyPose, visibleTags);

            // publish primary target data
            DetectedTag primary = visibleTags.get(0);
            publishPrimaryTarget(primary);
        }

        // update telemetry
        frameCount++;
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdateTime > 1.0) {
            SmartDashboard.putNumber("LimelightHardwareSim/FPS", frameCount / (now - lastUpdateTime));
            frameCount = 0;
            lastUpdateTime = now;
        }
        SmartDashboard.putNumber("LimelightHardwareSim/TagCount", visibleTags.size());
    }

    /**
     * Calculates the camera's 3D pose based on robot pose and mounting offsets.
     */
    private Pose3d calculateCameraPose(Pose2d robotPose) {
        double forward = cameraForwardOffset.getAsDouble();
        double side = cameraSideOffset.getAsDouble();
        double height = cameraHeight.getAsDouble();
        double pitchDeg = cameraPitch.getAsDouble();

        // transform from robot to camera
        Translation3d cameraTranslation = new Translation3d(forward, side, height);
        Rotation3d cameraRotation = new Rotation3d(0, Units.degreesToRadians(-pitchDeg), 0);
        Transform3d robotToCamera = new Transform3d(cameraTranslation, cameraRotation);

        // robot pose in 3D (on ground plane)
        Pose3d robotPose3d = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                0.0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

        return robotPose3d.transformBy(robotToCamera);
    }

    /**
     * Finds all tags visible from the current camera position.
     */
    private List<DetectedTag> findVisibleTags(Pose3d cameraPose, Pose2d robotPose) {
        List<DetectedTag> visible = new ArrayList<>();

        double maxDist = maxDetectionDistance.getAsDouble();
        double minDist = minDetectionDistance.getAsDouble();
        double hFov = horizontalFov.getAsDouble() / 2.0;  // half angle
        double vFov = verticalFov.getAsDouble() / 2.0;
        double maxAngle = maxTagAngle.getAsDouble();

        for (AprilTag tag : fieldLayout.getTags()) {
            Pose3d tagPose3d = tag.pose;

            // calculate distance from camera to tag
            double distToCamera = cameraPose.getTranslation().getDistance(tagPose3d.getTranslation());

            // check distance bounds
            if (distToCamera < minDist || distToCamera > maxDist) {
                continue;
            }

            // transform tag position to camera coordinate frame
            Translation3d tagInCamera = tagPose3d.getTranslation().minus(cameraPose.getTranslation());

            // rotate to camera frame (camera looks along +X after this transform)
            Rotation3d cameraRotInverse = cameraPose.getRotation().unaryMinus();
            Translation3d tagInCameraFrame = tagInCamera.rotateBy(cameraRotInverse);

            // check tag is in front of camera (positive X)
            if (tagInCameraFrame.getX() <= 0) {
                continue;
            }

            // calculate horizontal and vertical angles
            double tx = Math.toDegrees(Math.atan2(tagInCameraFrame.getY(), tagInCameraFrame.getX()));
            double ty = Math.toDegrees(Math.atan2(tagInCameraFrame.getZ(), tagInCameraFrame.getX()));

            // check within FOV
            if (Math.abs(tx) > hFov || Math.abs(ty) > vFov) {
                continue;
            }

            // check tag orientation (dot product between camera forward and tag normal)
            // tag normal points out from the tag face (the -X direction of tag pose)
            Translation3d tagNormal = new Translation3d(-1, 0, 0).rotateBy(tagPose3d.getRotation());
            Translation3d cameraForward = new Translation3d(1, 0, 0).rotateBy(cameraPose.getRotation());

            // dot product: 1 = facing directly at camera, 0 = perpendicular, -1 = facing away
            double dot = tagNormal.getX() * cameraForward.getX() +
                         tagNormal.getY() * cameraForward.getY() +
                         tagNormal.getZ() * cameraForward.getZ();

            // tag must face camera (negative dot means tag normal points toward camera)
            double angleDeg = Math.toDegrees(Math.acos(Math.abs(dot)));
            if (angleDeg > maxAngle) {
                continue;
            }

            // calculate area (simplified model based on distance and angle)
            double apparentSize = TAG_SIZE_METERS / distToCamera;
            double area = 100.0 * apparentSize * apparentSize * 50.0 * Math.abs(dot);
            area = Math.min(100.0, Math.max(0.0, area));

            // calculate ambiguity (higher when further away or at sharp angles)
            double ambiguity = 0.1 + 0.3 * (distToCamera / maxDist) + 0.3 * (1.0 - Math.abs(dot));
            ambiguity = Math.min(1.0, ambiguity);

            // distance from tag to robot center
            double distToRobot = robotPose.getTranslation().getDistance(
                    new Translation2d(tagPose3d.getX(), tagPose3d.getY()));

            visible.add(new DetectedTag(tag.ID, distToCamera, distToRobot, tx, ty, area, ambiguity, tagPose3d));

            if (verboseLogging) {
                Util.log("LimelightHardwareSim: tag %d visible at %.2fm, tx=%.1f ty=%.1f area=%.2f",
                        tag.ID, distToCamera, tx, ty, area);
            }
        }

        return visible;
    }

    /**
     * Adds Gaussian noise to the pose estimate.
     */
    private Pose2d addNoise(Pose2d pose) {
        double posNoise = positionNoiseStdDev.getAsDouble();
        double rotNoise = Units.degreesToRadians(rotationNoiseStdDev.getAsDouble());

        double noiseX = random.nextGaussian() * posNoise;
        double noiseY = random.nextGaussian() * posNoise;
        double noiseRot = random.nextGaussian() * rotNoise;

        return new Pose2d(
                pose.getX() + noiseX,
                pose.getY() + noiseY,
                pose.getRotation().plus(new Rotation2d(noiseRot)));
    }

    /**
     * Publishes the classic (MegaTag1) pose to NetworkTables.
     * <p>
     * Array format: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgDist, avgArea, ...fiducials]
     */
    private void publishClassicPose(Pose2d pose, List<DetectedTag> tags) {
        double latency = pipelineLatency.getAsDouble();

        // calculate aggregate metrics
        double avgDist = tags.stream().mapToDouble(t -> t.distanceToCamera).average().orElse(0);
        double avgArea = tags.stream().mapToDouble(t -> t.area).average().orElse(0);
        double tagSpan = calculateTagSpan(tags);

        // build array: 11 header values + 7 values per tag
        int arraySize = 11 + 7 * tags.size();
        double[] data = new double[arraySize];

        // header values
        data[0] = pose.getX();
        data[1] = pose.getY();
        data[2] = 0.0;  // z
        data[3] = 0.0;  // roll
        data[4] = 0.0;  // pitch
        data[5] = pose.getRotation().getDegrees();  // yaw
        data[6] = latency;
        data[7] = tags.size();
        data[8] = tagSpan;
        data[9] = avgDist;
        data[10] = avgArea;

        // per-tag fiducial data
        for (int i = 0; i < tags.size(); i++) {
            DetectedTag tag = tags.get(i);
            int base = 11 + i * 7;
            data[base] = tag.id;
            data[base + 1] = tag.tx;  // txnc
            data[base + 2] = tag.ty;  // tync
            data[base + 3] = tag.area;
            data[base + 4] = tag.distanceToCamera;
            data[base + 5] = tag.distanceToRobot;
            data[base + 6] = tag.ambiguity;
        }

        botposePublisher.set(data);
    }

    /**
     * Publishes the MegaTag2 pose to NetworkTables.
     * <p>
     * MegaTag2 uses the robot's gyro heading, so it should be more accurate.
     */
    private void publishMegaTag2Pose(Pose2d pose, List<DetectedTag> tags) {
        double latency = pipelineLatency.getAsDouble();

        // calculate aggregate metrics
        double avgDist = tags.stream().mapToDouble(t -> t.distanceToCamera).average().orElse(0);
        double avgArea = tags.stream().mapToDouble(t -> t.area).average().orElse(0);
        double tagSpan = calculateTagSpan(tags);

        // MegaTag2 typically has lower ambiguity since it uses gyro
        // reduce noise for multi-tag scenarios
        Pose2d mt2Pose = pose;
        if (tags.size() > 1) {
            // slightly less noisy for multi-tag
            double noiseReduction = 0.7;
            double dx = pose.getX() - (pose.getX() * noiseReduction);
            double dy = pose.getY() - (pose.getY() * noiseReduction);
            mt2Pose = new Pose2d(pose.getX() - dx * 0.5, pose.getY() - dy * 0.5, pose.getRotation());
        }

        // build array: 11 header values + 7 values per tag
        int arraySize = 11 + 7 * tags.size();
        double[] data = new double[arraySize];

        // header values
        data[0] = mt2Pose.getX();
        data[1] = mt2Pose.getY();
        data[2] = 0.0;  // z
        data[3] = 0.0;  // roll
        data[4] = 0.0;  // pitch
        data[5] = mt2Pose.getRotation().getDegrees();  // yaw
        data[6] = latency;
        data[7] = tags.size();
        data[8] = tagSpan;
        data[9] = avgDist;
        data[10] = avgArea;

        // per-tag fiducial data (with reduced ambiguity for MegaTag2)
        for (int i = 0; i < tags.size(); i++) {
            DetectedTag tag = tags.get(i);
            int base = 11 + i * 7;
            data[base] = tag.id;
            data[base + 1] = tag.tx;
            data[base + 2] = tag.ty;
            data[base + 3] = tag.area;
            data[base + 4] = tag.distanceToCamera;
            data[base + 5] = tag.distanceToRobot;
            data[base + 6] = tag.ambiguity * 0.5;  // lower ambiguity for MegaTag2
        }

        botposeOrbPublisher.set(data);
    }

    /**
     * Calculates the tag span (distance between furthest tags).
     */
    private double calculateTagSpan(List<DetectedTag> tags) {
        if (tags.size() < 2) {
            return 0;
        }

        double maxSpan = 0;
        for (int i = 0; i < tags.size(); i++) {
            for (int j = i + 1; j < tags.size(); j++) {
                Translation3d pos1 = tags.get(i).tagPose.getTranslation();
                Translation3d pos2 = tags.get(j).tagPose.getTranslation();
                double span = pos1.getDistance(pos2);
                maxSpan = Math.max(maxSpan, span);
            }
        }
        return maxSpan;
    }

    /**
     * Publishes primary target data (tx, ty, ta, tid).
     */
    private void publishPrimaryTarget(DetectedTag primary) {
        tvPublisher.set(1.0);
        txPublisher.set(primary.tx);
        tyPublisher.set(primary.ty);
        taPublisher.set(primary.area);
        tidPublisher.set(primary.id);
    }

    /**
     * Publishes empty data when no targets are visible.
     */
    private void publishNoTargets() {
        tvPublisher.set(0.0);
        txPublisher.set(0.0);
        tyPublisher.set(0.0);
        taPublisher.set(0.0);
        tidPublisher.set(-1);

        // publish empty pose arrays
        double[] emptyPose = new double[0];
        botposePublisher.set(emptyPose);
        botposeOrbPublisher.set(emptyPose);

        SmartDashboard.putNumber("LimelightHardwareSim/TagCount", 0);
    }
}

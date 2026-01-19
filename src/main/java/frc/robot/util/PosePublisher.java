package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Publishes position estimates in a format compatible with AdvantageScope's
 * Odometry tab.
 * <p>
 * Based on PosePublisher from Robohawk2022/2025CompetitionRobot.
 *
 * @see <a href="https://docs.advantagescope.org/tab-reference/odometry/">AdvantageScope Odometry Docs</a>
 */
public class PosePublisher {

    /** Pose with NaN values for when no data is available */
    public static final Pose2d NO_POSE = new Pose2d(
            Double.NaN,
            Double.NaN,
            Rotation2d.fromRadians(Double.NaN));

    private static final Map<String, StructPublisher<Pose2d>> publishers = new HashMap<>();

    /**
     * Publishes a pose to NetworkTables under "PoseEstimates/{name}".
     *
     * @param name the name of the pose (e.g., "Odometry", "Vision", "Fused")
     * @param pose the pose to publish
     */
    public static void publish(String name, Pose2d pose) {
        StructPublisher<Pose2d> publisher = publishers.get(name);
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                    .getStructTopic("PoseEstimates/" + name, Pose2d.struct)
                    .publish();
            publishers.put(name, publisher);
        }
        publisher.set(pose);
    }

    /**
     * Publishes that no pose data is available for a given name.
     *
     * @param name the name of the pose
     */
    public static void publishNone(String name) {
        publish(name, NO_POSE);
    }
}

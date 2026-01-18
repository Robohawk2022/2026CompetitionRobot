package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A pose with an associated timestamp, used for latency compensation
 * in odometry.
 *
 * @param pose the robot pose
 * @param timestamp the FPGA timestamp in seconds when this pose was recorded
 */
public record TimestampedPose(Pose2d pose, double timestamp) {

    /**
     * @return the age of this pose in seconds relative to the given time
     */
    public double getAge(double currentTime) {
        return currentTime - timestamp;
    }
}

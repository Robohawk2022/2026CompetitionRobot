package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.Util;

/**
 * Calculates a pose based on {@link ChassisSpeeds} updates for simulating
 * a swerve drive
 */
public class SwerveDeadReckoner {

    Pose2d latestPose;

    /**
     * Creates a {@link SwerveDeadReckoner}
     * @param initialPose initial robot pose
     */
    public SwerveDeadReckoner(Pose2d initialPose) {
        latestPose = initialPose;
    }

    /**
     * @return the current calculated pose of the robot
     */
    public Pose2d getPose() {
        return latestPose;
    }

    /**
     * Updates calculated position
     * @param speeds robot-relative movement speeds
     */
    public void update(ChassisSpeeds speeds) {

        if (speeds != null) {

            // convert robot-relative to field-relative for updating
            // field position
            speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    speeds,
                    latestPose.getRotation());

            double dx = speeds.vxMetersPerSecond * Util.DT;
            double dy = speeds.vyMetersPerSecond * Util.DT;
            double dh = speeds.omegaRadiansPerSecond * Util.DT;

            // calculate a new pose estimate based on the supplied speeds
            latestPose = new Pose2d(
                    latestPose.getX() + dx,
                    latestPose.getY() + dy,
                    Rotation2d.fromRadians(latestPose.getRotation().getRadians() + dh));
        }
    }

    public void resetPose(Pose2d pose) {
        latestPose = pose;
    }
}

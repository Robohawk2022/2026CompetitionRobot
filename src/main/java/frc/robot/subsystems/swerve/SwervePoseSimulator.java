package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.Util;

/**
 * Calculates a pose based on {@link ChassisSpeeds} updates for simulating
 * a swerve drive
 */
public class SwervePoseSimulator {

    double x;
    double y;
    double heading;

    /**
     * Creates a {@link SwervePoseSimulator}
     * @param initialPose initial robot pose
     */
    public SwervePoseSimulator(Pose2d initialPose) {
        resetPose(initialPose);
    }

    /**
     * @return the current calculated pose of the robot
     */
    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromRadians(heading));
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
                    Rotation2d.fromRadians(heading));

            x += speeds.vxMetersPerSecond * Util.DT;
            y += speeds.vyMetersPerSecond * Util.DT;
            heading += speeds.omegaRadiansPerSecond * Util.DT;
        }
    }

    public void resetPose(Pose2d pose) {
        x = pose.getTranslation().getX();
        y = pose.getTranslation().getY();
        heading = pose.getRotation().getRadians();
    }
}

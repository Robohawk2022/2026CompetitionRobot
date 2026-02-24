package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.Util;

/**
 * Calculates a pose based on ChassisSpeed updates
 */
public class SwervePoseCalculator {

    double x;
    double y;
    double heading;

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromRadians(heading));
    }

    public void update(ChassisSpeeds speeds) {
        if (speeds != null) {
            x += speeds.vxMetersPerSecond * Util.DT;
            y += speeds.vyMetersPerSecond * Util.DT;
            heading += speeds.omegaRadiansPerSecond * Util.DT;
        }
    }

    public void reset(Pose2d pose) {
        x = pose.getTranslation().getX();
        y = pose.getTranslation().getY();
        heading = pose.getRotation().getRadians();
    }
}

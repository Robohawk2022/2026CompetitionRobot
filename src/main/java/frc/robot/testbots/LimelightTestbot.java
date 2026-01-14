package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

import static frc.robot.Config.Limelight.limelightName;

/**
 * Testbot for vision-related functionality.
 */
public class LimelightTestbot extends TimedRobot {

    boolean logTargeting = true;
    boolean logAprilTags = true;

    public LimelightTestbot() {

        SmartDashboard.putData("LimelightTestbot", builder -> {

            // this should be the information we can get about the "current
            // target" - the docs are a little bit light as to what this means
            // if there are multiple tags in view
            builder.addDoubleProperty("Target/Count", () -> LimelightHelpers.getTargetCount(limelightName), null);
            builder.addDoubleProperty("Target/TagId", () -> LimelightHelpers.getFiducialID(limelightName), null);
            builder.addDoubleProperty("Target/TA", () -> LimelightHelpers.getTA(limelightName), null);
            builder.addDoubleProperty("Target/TX", () -> LimelightHelpers.getTX(limelightName), null);
            builder.addDoubleProperty("Target/TXNC", () -> LimelightHelpers.getTXNC(limelightName), null);
            builder.addDoubleProperty("Target/TYNC", () -> LimelightHelpers.getTYNC(limelightName), null);

            // this should be a list of the currently in-view AprilTags
            builder.addStringProperty("AprilTagIds", this::getAprilTagIds, null);

        });
    }

    /**
     * @return a comma-separated list of the AprilTags that are currently
     * in view
     */
    public String getAprilTagIds() {

        RawFiducial [] tags = LimelightHelpers.getRawFiducials(limelightName);
        if (tags.length == 0) {
            return "(none)";
        }

        StringBuilder builder = new StringBuilder();
        for (RawFiducial tag : tags) {
            if (!builder.isEmpty()) {
                builder.append(", ");
            }
            builder.append(tag.id);
        }
        return builder.toString();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

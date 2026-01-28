package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Config.Swerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Field;
import frc.robot.util.Util;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.Objects;

import static frc.robot.Config.QuestNav.questToRobotTransform;
import static frc.robot.Config.QuestNav.robotToQuestTransform;
import static frc.robot.Config.QuestNav.enabled;

/**
 * Subsystem that uses the QuestNav to generate pose estimates during
 * <code>periodic</code> and, if they pass relevant quality checks, feed them
 * to a consumer (e.g. the swerve drive)
 */
public class QuestNavSubsystem extends SubsystemBase {

    final SwerveSubsystem swerve;
    final QuestNav quest;
    boolean estimateProvided;
    long noPose;
    long poseOutsideField;
    long poseJumpTooBig;
    long highConfidence;

    /**
     * Creates a {@link QuestNavSubsystem}
     * @param swerve the swerve drive
     * @throws IllegalArgumentException if required parameters are null
     */
    public QuestNavSubsystem(SwerveSubsystem swerve) {

        this.swerve = Objects.requireNonNull(swerve);
        this.quest = new QuestNav();
        this.noPose = 0L;
        this.poseOutsideField = 0L;
        this.poseJumpTooBig = 0L;
        this.highConfidence = 0L;

        SmartDashboard.putData("QuestNav", builder -> {
            builder.addBooleanProperty("IsConnected?", quest::isConnected, null);
            builder.addBooleanProperty("IsTracking?", quest::isConnected, null);
            builder.addBooleanProperty("HasEstimate?", () -> estimateProvided, null);
            builder.addIntegerProperty("Metadata/Battery", () -> quest.getBatteryPercent().orElse(-1), null);
            builder.addStringProperty("Metadata/Version", quest::getLibVersion, null);
            builder.addDoubleProperty("Metadata/Latency", quest::getLatency, null);
            builder.addDoubleProperty("Results/1-NoPose", () -> noPose, null);
            builder.addDoubleProperty("Results/2-PoseOutsideField", () -> poseOutsideField, null);
            builder.addDoubleProperty("Results/2-PoseJumpTooBig", () -> poseJumpTooBig, null);
            builder.addDoubleProperty("Results/3-HighConfidence", () -> highConfidence, null);
        });
    }

    /**
     * QuestNav uses dead reckoning to estimate the headset's position, so it's
     * important to reset it at an accurate, known location on the field
     * whenever one becomes available. See <a href="https://questnav.gg/docs/getting-started/robot-code#setting-robot-pose">vendor
     * documentation here</a>.
     *
     * @param pose the robot's pose on the field
     */
    public void resetPose(Pose2d pose) {
        Pose3d robotPose = new Pose3d(pose);
        Pose3d questPose = robotPose.transformBy(robotToQuestTransform);
        quest.setPose(questPose);
    }

    /**
     * Updates pose information from the headset; see <a href="https://questnav.gg/docs/getting-started/robot-code#getting-robot-pose">vendor
     * documentation here</a>.
     */
    @Override
    public void periodic() {

        // if we're enabled, poll the QuestNav and make a pose estimate
        Pose2d lastEstimate = null;
        if (enabled.getAsBoolean()) {
            quest.commandPeriodic();
            lastEstimate = attemptPoseEstimate();
        }

        // publish estimates as appropriate
        if (lastEstimate != null) {
            Util.publishPose("QuestNav", lastEstimate);
            estimateProvided = true;
        } else {
            Util.publishPose("QuestNav", Util.NAN_POSE);
            estimateProvided = false;
        }
    }

    /**
     * Calculates a pose estimate using the latest information from the
     * headset, passes it through some quality gates, and possibly supplies it
     * to the drive
     * @return the pose we sent (null if we didn't have one)
     */
    private Pose2d attemptPoseEstimate() {

        Pose3d questPose = null;
        double questTime = Double.NaN;

        // let's see if we have an estimate
        PoseFrame [] poseFrames = quest.getAllUnreadPoseFrames();
        if (poseFrames != null && poseFrames.length > 0) {
            questPose = poseFrames[poseFrames.length - 1].questPose3d();
            questTime = poseFrames[poseFrames.length - 1].dataTimestamp();
        }

        // if we don't have an estimate, we've got nothing to do
        if (questPose == null) {
            noPose++;
            return null;
        }

        Pose2d currentPose = swerve.getPose();
        Pose2d estimatedPose = questPose
                .transformBy(questToRobotTransform)
                .toPose2d();

        // if the pose is outside the field, there's no use dealing with it
        if (Field.isOutsideField(estimatedPose)) {
            poseOutsideField++;
            return null;
        }

        // if the pose is too far from the current pose, we don't submit it
        if (Util.feetBetween(currentPose, estimatedPose) > Swerve.maxPoseJumpFeet.getAsDouble()) {
            poseJumpTooBig++;
            return null;
        }

        // we will always report the same level of confidence
        highConfidence++;
        swerve.submitVisionEstimate(estimatedPose, questTime, Config.QuestNav.confidence);
        return estimatedPose;
    }
}

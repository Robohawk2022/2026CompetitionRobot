package frc.robot.subsystems.limelight;

import edu.wpi.first.util.sendable.SendableBuilder;

public class LimelightResults {

    long noEstimate;
    long noTags;
    long poseOutsideField;
    long poseJumpTooBig;
    long lowConfidence;
    long highConfidence;
    long mediumConfidence;
    boolean validEstimate;

    public void addToDashboard(SendableBuilder builder) {
        builder.addIntegerProperty("Results/ErrNoEstimate", () -> noEstimate, null);
        builder.addIntegerProperty("Results/ErrNoTags", () -> noEstimate, null);
        builder.addIntegerProperty("Results/ErrOutsideField", () -> poseOutsideField, null);
        builder.addIntegerProperty("Results/ErrTooFar", () -> poseJumpTooBig, null);
        builder.addIntegerProperty("Results/ConfLow", () -> lowConfidence, null);
        builder.addIntegerProperty("Results/ConfMed", () -> mediumConfidence, null);
        builder.addIntegerProperty("Results/ConfHigh", () -> highConfidence, null);
    }

    public void noEstimate() {
        noEstimate++;
        validEstimate = false;
    }
    
    public void noTags() {
        noTags++;
        validEstimate = false;
    }
    
    public void poseOutsideField() {
        poseOutsideField++;
        validEstimate = false;
    }
    
    public void poseJumpTooBig() {
        poseJumpTooBig++;
        validEstimate = false;
    }
    
    public void lowConfidence() {
        lowConfidence++;
        validEstimate = true;
    }
    
    public void highConfidence() {
        highConfidence++;
        validEstimate = true;
    }
    
    public void mediumConfidence() {
        mediumConfidence++;
        validEstimate = true;
    }
}

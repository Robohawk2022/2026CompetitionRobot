package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;

/**
 * Hardware configuration constants for the swerve drive.
 * <p>
 * Contains CAN IDs, angular offsets, motor inversions, and PID gains.
 * These values are read once at startup and require a code redeploy to change.
 */
public final class SwerveHardwareConfig {

    private SwerveHardwareConfig() {
        // prevent instantiation
    }

    //===========================================================================
    // CAN IDs - Module order: FL (Front Left), FR, BL, BR
    //===========================================================================

    public static final int FL_DRIVE_ID = 41;
    public static final int FL_TURN_ID = 42;
    public static final int FL_ENCODER_ID = 43;

    public static final int FR_DRIVE_ID = 21;
    public static final int FR_TURN_ID = 22;
    public static final int FR_ENCODER_ID = 23;

    public static final int BL_DRIVE_ID = 11;
    public static final int BL_TURN_ID = 12;
    public static final int BL_ENCODER_ID = 13;

    public static final int BR_DRIVE_ID = 31;
    public static final int BR_TURN_ID = 32;
    public static final int BR_ENCODER_ID = 33;

    public static final int PIGEON_ID = 10;

    //===========================================================================
    // Module angular offsets (radians)
    // Calibrate by pointing all wheels forward and reading CANcoder values,
    // then negate them here
    //===========================================================================

    public static final double FL_ANGULAR_OFFSET = 0;
    public static final double FR_ANGULAR_OFFSET = 0;
    public static final double BL_ANGULAR_OFFSET = 0;
    public static final double BR_ANGULAR_OFFSET = 0;

    //===========================================================================
    // Motor inversions
    // The modules on one side of the robot are inverted with respect to the
    // other, so we need to make sure the drive motors spin in the correct
    // direction
    //===========================================================================

    public static final InvertedValue FL_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FR_DRIVE_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue BL_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue BR_DRIVE_INVERTED = InvertedValue.CounterClockwise_Positive;

    //===========================================================================
    // Drive motor PID (velocity control)
    //===========================================================================

    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KV = 0.12;
    public static final double DRIVE_KS = 0.0;

    //===========================================================================
    // Turn motor PID (position control)
    //===========================================================================

    public static final double TURN_KP = 6.0;
    public static final double TURN_KD = 0.001;
}

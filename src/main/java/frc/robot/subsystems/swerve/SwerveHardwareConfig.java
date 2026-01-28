package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Hardware configuration constants for the swerve drive.
 * <p>
 * Contains CAN IDs, angular offsets, motor inversions, and PID gains.
 * These values are read once at startup and require a code redeploy to change.
 */
public final class SwerveHardwareConfig {

    //==========================================================================
    // GENERAL ROBOT & MOTORS
    //==========================================================================

    /** Robot mass including bumpers and battery (kg) */
    public static final double ROBOT_MASS_KG = 54.0;

    /** Robot moment of inertia (kg*m^2) - estimate or calculate from CAD */
    public static final double ROBOT_MOI = 6.0;

    /** General motor properties */
    public static final double MOTOR_FREE_SPEED_RPM = 6380.0;
    public static final double MOTOR_STALL_TORQUE_NM = 7.09;
    public static final double MOTOR_STALL_CURRENT_AMPS = 366.0;

    //==========================================================================
    // CHASSIS MEASUREMENTS
    //==========================================================================

    public static final double WHEEL_BASE_INCHES = 19.0 + 7.0 / 16.0;
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(WHEEL_BASE_INCHES);
    public static final double TRACK_WIDTH_INCHES = 25.0;
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),   // FL
            new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),  // FR
            new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),  // BL
            new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)  // BR
    );

    //==========================================================================
    // WHEELS & MOTOR GEARING
    //==========================================================================

    // wheel size & friction
    public static final double WHEEL_DIAMETER_METERS = 0.1016;  // 4 inch wheels
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double WHEEL_COF = 1.0;

    // manufacturer-supplied gear ratios - see https://www.swervedrivespecialties.com/products/mk5i-swerve-module
    public static final double SDS_MK5I_DRIVE_R1 = 7.03;
    public static final double SDS_MK5I_DRIVE_R2 = 6.03;
    public static final double SDS_MK5I_DRIVE_R3 = 5.27;
    public static final double SDS_MK5I_TURN = 26.0;

    // drive motor
    public static final double DRIVE_GEAR_RATIO = SDS_MK5I_DRIVE_R3;
    public static final double DRIVE_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_FACTOR / 60.0;
    public static final double DRIVE_CURRENT_LIMIT_AMPS = 60.0;

    // turn motor
    public static final double TURN_GEAR_RATIO =  SDS_MK5I_TURN;
    public static final double TURN_CURRENT_LIMIT_AMPS = 40.0;

    public static final double MAX_MOTOR_RPM = 6380.0;  // Kraken X60 free speed
    public static final double MAX_WHEEL_SPEED_MPS = (MAX_MOTOR_RPM / 60.0) * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

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

    //==========================================================================
    // MOTOR_INVERSIONS
    //==========================================================================

    public static final InvertedValue FL_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FR_DRIVE_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue BL_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue BR_DRIVE_INVERTED = InvertedValue.CounterClockwise_Positive;

    //==========================================================================
    // PID TUNING
    //==========================================================================

    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KV = 0.12;
    public static final double DRIVE_KS = 0.0;

    public static final double TURN_KP = 6.0;
    public static final double TURN_KD = 0.001;
}

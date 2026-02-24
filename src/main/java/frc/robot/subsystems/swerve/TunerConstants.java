package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

/**
 * Swerve drivetrain constants generated/adapted from Tuner X.
 * <p>
 * Contains all hardware configuration for CTRE's swerve drive implementation.
 * CAN IDs match the existing SwerveHardwareConfig values.
 */
public class TunerConstants {

    // PID gains for steer motors (tuned for CTRE swerve)
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(SwerveHardwareConfig.TURN_KP)
            .withKI(0)
            .withKD(SwerveHardwareConfig.TURN_KD)
            .withKS(SwerveHardwareConfig.TURN_KS)
            .withKV(SwerveHardwareConfig.TURN_KV)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // PID gains for drive motors (tuned for CTRE swerve)
    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(SwerveHardwareConfig.DRIVE_KP)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(SwerveHardwareConfig.DRIVE_KV);

    // Closed-loop output types
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // Motor arrangements
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // Feedback type for steer motors (FusedCANcoder for best accuracy)
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // Slip current - the stator current at which wheels start to slip
    private static final Current kSlipCurrent = Amps.of(120);

    // Initial motor configs
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(SwerveHardwareConfig.TURN_CURRENT_LIMIT_AMPS))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus configuration
    public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

    // Theoretical free speed at 12V (Kraken X60 with MK5i R3 gearing)
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.85);

    // Coupling ratio: every 1 rotation of azimuth results in this many drive motor turns
    private static final double kCoupleRatio = 3.375;

    // Gear ratios (MK5i R3)
    private static final double kDriveGearRatio = SwerveHardwareConfig.DRIVE_GEAR_RATIO;
    private static final double kSteerGearRatio = SwerveHardwareConfig.TURN_GEAR_RATIO;
    private static final Distance kWheelRadius = Inches.of(2);

    // Motor inversions
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    // Pigeon ID
    private static final int kPigeonId = SwerveHardwareConfig.PIGEON_ID;

    // Simulation parameters
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    // Drivetrain constants
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    // Module constant factory
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    //==========================================================================
    // Front Left Module
    //==========================================================================
    private static final int kFrontLeftDriveMotorId = SwerveHardwareConfig.FL_DRIVE_ID;
    private static final int kFrontLeftSteerMotorId = SwerveHardwareConfig.FL_TURN_ID;
    private static final int kFrontLeftEncoderId = SwerveHardwareConfig.FL_ENCODER_ID;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.2177734375);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;
    private static final Distance kFrontLeftXPos = Inches.of(SwerveHardwareConfig.WHEEL_BASE_INCHES / 2.0);
    private static final Distance kFrontLeftYPos = Inches.of(SwerveHardwareConfig.TRACK_WIDTH_INCHES / 2.0);

    //==========================================================================
    // Front Right Module
    //==========================================================================
    private static final int kFrontRightDriveMotorId = SwerveHardwareConfig.FR_DRIVE_ID;
    private static final int kFrontRightSteerMotorId = SwerveHardwareConfig.FR_TURN_ID;
    private static final int kFrontRightEncoderId = SwerveHardwareConfig.FR_ENCODER_ID;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.19775390625);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;
    private static final Distance kFrontRightXPos = Inches.of(SwerveHardwareConfig.WHEEL_BASE_INCHES / 2.0);
    private static final Distance kFrontRightYPos = Inches.of(-SwerveHardwareConfig.TRACK_WIDTH_INCHES / 2.0);

    //==========================================================================
    // Back Left Module
    //==========================================================================
    private static final int kBackLeftDriveMotorId = SwerveHardwareConfig.BL_DRIVE_ID;
    private static final int kBackLeftSteerMotorId = SwerveHardwareConfig.BL_TURN_ID;
    private static final int kBackLeftEncoderId = SwerveHardwareConfig.BL_ENCODER_ID;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.051025390625);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;
    private static final Distance kBackLeftXPos = Inches.of(-SwerveHardwareConfig.WHEEL_BASE_INCHES / 2.0);
    private static final Distance kBackLeftYPos = Inches.of(SwerveHardwareConfig.TRACK_WIDTH_INCHES / 2.0);

    //==========================================================================
    // Back Right Module
    //==========================================================================
    private static final int kBackRightDriveMotorId = SwerveHardwareConfig.BR_DRIVE_ID;
    private static final int kBackRightSteerMotorId = SwerveHardwareConfig.BR_TURN_ID;
    private static final int kBackRightEncoderId = SwerveHardwareConfig.BR_ENCODER_ID;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.169189453125);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;
    private static final Distance kBackRightXPos = Inches.of(-SwerveHardwareConfig.WHEEL_BASE_INCHES / 2.0);
    private static final Distance kBackRightYPos = Inches.of(-SwerveHardwareConfig.TRACK_WIDTH_INCHES / 2.0);

    //==========================================================================
    // Module Constants
    //==========================================================================
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}

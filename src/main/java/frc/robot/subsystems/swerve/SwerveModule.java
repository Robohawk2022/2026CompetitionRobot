package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Config.Swerve.*;

/**
 * Represents a single swerve module with CTRE TalonFX drive and turn motors,
 * plus a CANcoder for absolute position sensing.
 */
public class SwerveModule {

    /** Update frequency for high-frequency odometry signals (Hz) */
    private static final double HF_UPDATE_FREQUENCY = 250.0;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnEncoder;

    private final double angularOffset;
    private final String name;

    private final VelocityVoltage driveRequest = new VelocityVoltage(0);
    private final PositionVoltage turnRequest = new PositionVoltage(0);

    private SwerveModuleState desiredState = new SwerveModuleState();

    // status signals for high-frequency odometry
    private final StatusSignal<Angle> drivePositionSignal;
    private final StatusSignal<Angle> turnPositionSignal;

    /**
     * Creates a new SwerveModule.
     *
     * @param name          module name for dashboard (e.g., "FL", "FR")
     * @param driveId       CAN ID of the drive motor
     * @param turnId        CAN ID of the turn motor
     * @param encoderId     CAN ID of the CANcoder
     * @param angularOffset angular offset of the module in radians
     */
    public SwerveModule(String name, int driveId, int turnId, int encoderId, double angularOffset, InvertedValue driveInvertedValue) {
        this.name = name;
        this.angularOffset = angularOffset;

        driveMotor = new TalonFX(driveId);
        turnMotor = new TalonFX(turnId);
        turnEncoder = new CANcoder(encoderId);

        configureDriveMotor(driveInvertedValue);
        configureTurnMotor();
        // configureCANcoder();

        // store status signals for high-frequency odometry
        drivePositionSignal = driveMotor.getPosition();
        turnPositionSignal = turnEncoder.getAbsolutePosition();

        // set update frequency for high-frequency odometry (250Hz)
        drivePositionSignal.setUpdateFrequency(HF_UPDATE_FREQUENCY);
        turnPositionSignal.setUpdateFrequency(HF_UPDATE_FREQUENCY);

        // initialize desired state to current position
        desiredState.angle = Rotation2d.fromRadians(getTurnPositionRadians());

        // dashboard telemetry
        SmartDashboard.putData("Swerve/Module-" + name, builder -> {
            builder.addDoubleProperty("Velocity", () -> getState().speedMetersPerSecond, null);
            builder.addDoubleProperty("Angle", () -> getState().angle.getDegrees(), null);
            builder.addDoubleProperty("DesiredVelocity", () -> desiredState.speedMetersPerSecond, null);
            builder.addDoubleProperty("DesiredAngle", () -> desiredState.angle.getDegrees(), null);
        });
    }

    private void configureDriveMotor(InvertedValue driveInvertedValue) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = driveInvertedValue;

        // current limits
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // feedback - rotor sensor (internal encoder)
        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        // PID for velocity control
        config.Slot0.kP = driveKP.getAsDouble();
        config.Slot0.kV = driveKV.getAsDouble();
        config.Slot0.kS = driveKS.getAsDouble();

        driveMotor.getConfigurator().apply(config);
    }

    private void configureTurnMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // current limits
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // use remote CANcoder for feedback
        config.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = TURN_GEAR_RATIO;

        // PID for position control
        config.Slot0.kP = turnKP.getAsDouble();
        config.Slot0.kD = turnKD.getAsDouble();

        // enable continuous input for wrap-around
        config.ClosedLoopGeneral.ContinuousWrap = true;

        turnMotor.getConfigurator().apply(config);
    }

    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -angularOffset / (2 * Math.PI);
        turnEncoder.getConfigurator().apply(config);
    }

    /**
     * @return the current turn position in radians
     */
    private double getTurnPositionRadians() {
        return turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * @return the current state of the module (velocity + angle)
     */
    public SwerveModuleState getState() {
        double velocity = driveMotor.getVelocity().getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
        Rotation2d angle = Rotation2d.fromRadians(getTurnPositionRadians());
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * @return the current position of the module (distance + angle)
     */
    public SwerveModulePosition getPosition() {
        double distance = driveMotor.getPosition().getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
        Rotation2d angle = Rotation2d.fromRadians(getTurnPositionRadians());
        return new SwerveModulePosition(distance, angle);
    }

    /**
     * Sets the desired state for this module.
     *
     * @param state the desired state (velocity + angle)
     */
    public void setDesiredState(SwerveModuleState state) {
        // optimize the state to avoid spinning more than 90 degrees
       
        state.optimize(Rotation2d.fromRadians(getTurnPositionRadians()));

        // set drive velocity (in rotations per second)
        double velocityRps = state.speedMetersPerSecond / WHEEL_CIRCUMFERENCE_METERS;
        System.out.println(velocityRps);
        driveMotor.setControl(driveRequest.withVelocity(velocityRps));

        // set turn position (in rotations)
        double positionRotations = state.angle.getRotations();
        System.out.println(positionRotations);
        turnMotor.setControl(turnRequest.withPosition(positionRotations));

        desiredState = state;
      
    }

    /**
     * Resets the drive encoder to zero.
     */
    public void resetEncoder() {
        driveMotor.setPosition(0);
    }

    /**
     * @return the drive position status signal for synchronized reads
     */
    public StatusSignal<Angle> getDrivePositionSignal() {
        return drivePositionSignal;
    }

    /**
     * @return the turn position status signal for synchronized reads
     */
    public StatusSignal<Angle> getTurnPositionSignal() {
        return turnPositionSignal;
    }

    /**
     * @return all status signals for this module (drive position, turn position)
     */
    public BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[] { drivePositionSignal, turnPositionSignal };
    }
}

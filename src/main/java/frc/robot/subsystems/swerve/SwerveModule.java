package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Config.Swerve.*;
import static frc.robot.subsystems.swerve.SwerveHardwareConfig.*;

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

    // CAN IDs for verbose logging
    private final int driveId;
    private final int turnId;
    private final int encoderId;

    private final VelocityVoltage driveRequest = new VelocityVoltage(0);
    private final PositionVoltage turnRequest = new PositionVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);

    private SwerveModuleState desiredState = new SwerveModuleState();

    // status signals for high-frequency odometry (cached, refreshed once per cycle)
    private final StatusSignal<Angle> drivePositionSignal;
    private final StatusSignal<AngularVelocity> driveVelocitySignal;
    private final StatusSignal<Angle> turnPositionSignal;

    // cached values (updated in refreshSignals)
    private double cachedDrivePositionMeters;
    private double cachedDriveVelocityMps;
    private double cachedTurnPositionRad;

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
        this.driveId = driveId;
        this.turnId = turnId;
        this.encoderId = encoderId;

        driveMotor = new TalonFX(driveId);
        turnMotor = new TalonFX(turnId);
        turnEncoder = new CANcoder(encoderId);

        configureDriveMotor(driveInvertedValue);
        configureTurnMotor();
        // configureCANcoder();

        // store status signals for high-frequency odometry
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        turnPositionSignal = turnEncoder.getAbsolutePosition();

        // set update frequency for high-frequency odometry (250Hz)
        drivePositionSignal.setUpdateFrequency(HF_UPDATE_FREQUENCY);
        driveVelocitySignal.setUpdateFrequency(HF_UPDATE_FREQUENCY);
        turnPositionSignal.setUpdateFrequency(HF_UPDATE_FREQUENCY);

        // initialize cached values
        refreshSignals();

        // initialize desired state to current position
        desiredState.angle = Rotation2d.fromRadians(getTurnPositionRadians());

        // dashboard telemetry (uses cached values to avoid allocations)
        SmartDashboard.putData("SwerveModules/" + name, builder -> {
            builder.addDoubleProperty("VelocityCurrent", () -> Units.metersToFeet(cachedDriveVelocityMps), null);
            builder.addDoubleProperty("VelocityDesired", () -> Units.metersToFeet(desiredState.speedMetersPerSecond), null);
            builder.addDoubleProperty("VelocityError",
                () -> Units.metersToFeet(desiredState.speedMetersPerSecond - cachedDriveVelocityMps), null);
            builder.addDoubleProperty("AngleCurrent", () -> Math.toDegrees(cachedTurnPositionRad), null);
            builder.addDoubleProperty("AngleDesired", () -> desiredState.angle.getDegrees(), null);
            builder.addDoubleProperty("AngleError",
                () -> desiredState.angle.getDegrees() - Math.toDegrees(cachedTurnPositionRad), null);
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
        config.Slot0.kP = DRIVE_KP;
        config.Slot0.kV = DRIVE_KV;
        config.Slot0.kS = DRIVE_KS;

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
        config.Slot0.kP = TURN_KP;
        config.Slot0.kD = TURN_KD;

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
     * Refreshes cached values from the status signals.
     * Call this once per cycle after BaseStatusSignal.refreshAll().
     */
    public void refreshSignals() {
        cachedDrivePositionMeters = drivePositionSignal.getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
        cachedDriveVelocityMps = driveVelocitySignal.getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
        cachedTurnPositionRad = turnPositionSignal.getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * @return the current turn position in radians (cached)
     */
    private double getTurnPositionRadians() {
        return cachedTurnPositionRad;
    }

    /**
     * @return the current state of the module (velocity + angle) using cached values
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(cachedDriveVelocityMps, Rotation2d.fromRadians(cachedTurnPositionRad));
    }

    /**
     * @return the current position of the module (distance + angle) using cached values
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(cachedDrivePositionMeters, Rotation2d.fromRadians(cachedTurnPositionRad));
    }

    /**
     * Sets the desired state for this module.
     *
     * @param state the desired state (velocity + angle)
     */
    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = Rotation2d.fromRadians(getTurnPositionRadians());

        // if drive speed is below threshold, don't move the angle motor
        // this prevents jitter when stationary
        if (Math.abs(state.speedMetersPerSecond) < minSpeedForAngle.getAsDouble()) {
            driveMotor.setControl(driveRequest.withVelocity(0));
            desiredState = new SwerveModuleState(0, currentAngle);
            return;
        }

        // optimize the state to avoid spinning more than 90 degrees
        state.optimize(currentAngle);

        // apply cosine compensation: scale drive output by cos(angle error)
        // this accounts for the fact that only part of the wheel's velocity
        // contributes to the desired direction when the wheel is misaligned
        double speedMps = state.speedMetersPerSecond;
        if (cosineCompensation.getAsBoolean()) {
            double angleErrorRad = state.angle.minus(currentAngle).getRadians();
            speedMps *= Math.cos(angleErrorRad);
        }

        // set drive velocity (in rotations per second)
        double velocityRps = speedMps / WHEEL_CIRCUMFERENCE_METERS;
        driveMotor.setControl(driveRequest.withVelocity(velocityRps));

        // set turn position (in rotations)
        double positionRotations = state.angle.getRotations();
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
     * @return all status signals for this module (drive position, velocity, turn position)
     */
    public BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[] { drivePositionSignal, driveVelocitySignal, turnPositionSignal };
    }

    //==========================================================================
    // SysId characterization methods
    //==========================================================================

    /**
     * Applies raw voltage to the drive motor (for SysId characterization).
     * Bypasses velocity PID control.
     *
     * @param volts the voltage to apply
     */
    public void setDriveVoltage(double volts) {
        driveMotor.setControl(driveVoltageRequest.withOutput(volts));
    }

    /**
     * Sets the turn motor to a specific angle (for locking during SysId).
     *
     * @param angle the target angle
     */
    public void setTurnAngle(Rotation2d angle) {
        turnMotor.setControl(turnRequest.withPosition(angle.getRotations()));
    }

    /**
     * @return drive motor position in meters (cached value)
     */
    public double getDrivePositionMeters() {
        return cachedDrivePositionMeters;
    }

    /**
     * @return drive motor velocity in meters per second (cached value)
     */
    public double getDriveVelocityMps() {
        return cachedDriveVelocityMps;
    }
}

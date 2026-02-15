package frc.robot.subsystems.launcher;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Config.Launcher.*;

/**
 * Implements {@link LauncherHardware} using REV SparkMax motors (NEO)
 * with onboard closed-loop velocity control.
 */
public class LauncherHardwareRev implements LauncherHardware {

    private final SparkMax lowerWheelMotor;
    private final SparkMax upperWheelMotor;

    private final RelativeEncoder lowerWheelEncoder;
    private final RelativeEncoder upperWheelEncoder;

    private final SparkClosedLoopController lowerController;
    private final SparkClosedLoopController upperController;

    private final SparkMaxConfig lowerConfig;
    private final SparkMaxConfig upperConfig;

    public LauncherHardwareRev() {
        lowerWheelMotor = new SparkMax(LOWER_WHEEL_CAN_ID, MotorType.kBrushless);
        upperWheelMotor = new SparkMax(UPPER_WHEEL_CAN_ID, MotorType.kBrushless);

        lowerWheelEncoder = lowerWheelMotor.getEncoder();
        upperWheelEncoder = upperWheelMotor.getEncoder();

        lowerController = lowerWheelMotor.getClosedLoopController();
        upperController = upperWheelMotor.getClosedLoopController();

        // separate PID per wheel
        lowerConfig = createWheelConfig(lowerWheelInverted.getAsBoolean(),
                lowerKV.getAsDouble(), lowerKP.getAsDouble());
        lowerWheelMotor.configure(lowerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        upperConfig = createWheelConfig(upperWheelInverted.getAsBoolean(),
                upperKV.getAsDouble(), upperKP.getAsDouble());
        upperWheelMotor.configure(upperConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    private SparkMaxConfig createWheelConfig(boolean inverted, double kV, double kP) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());
        config.closedLoopRampRate(0.1);

        config.closedLoop.p(kP);
        config.closedLoop.feedForward.kV(kV);

        return config;
    }

    @Override
    public void setLowerWheelRPM(double rpm) {
        if (rpm == 0) {
            lowerWheelMotor.set(0);
        } else {
            lowerController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    @Override
    public void setUpperWheelRPM(double rpm) {
        if (rpm == 0) {
            upperWheelMotor.set(0);
        } else {
            upperController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    @Override
    public void resetLowerPID(double kV, double kP, double kI, double kD) {
        lowerConfig.inverted(lowerWheelInverted.getAsBoolean());
        lowerConfig.closedLoop.p(kP);
        lowerConfig.closedLoop.i(kI);
        lowerConfig.closedLoop.d(kD);
        lowerConfig.closedLoop.feedForward.kV(kV);
        lowerWheelMotor.configure(lowerConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void resetUpperPID(double kV, double kP, double kI, double kD) {
        upperConfig.inverted(upperWheelInverted.getAsBoolean());
        upperConfig.closedLoop.p(kP);
        upperConfig.closedLoop.i(kI);
        upperConfig.closedLoop.d(kD);
        upperConfig.closedLoop.feedForward.kV(kV);
        upperWheelMotor.configure(upperConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public double getLowerWheelRPM() {
        return lowerWheelEncoder.getVelocity();
    }

    @Override
    public double getUpperWheelRPM() {
        return upperWheelEncoder.getVelocity();
    }

    @Override
    public double getLowerWheelAmps() {
        return lowerWheelMotor.getOutputCurrent();
    }

    @Override
    public double getUpperWheelAmps() {
        return upperWheelMotor.getOutputCurrent();
    }
}

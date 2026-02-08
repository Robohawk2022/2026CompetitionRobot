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
 * with onboard closed-loop velocity control for the wheels.
 */
public class LauncherHardwareRev implements LauncherHardware {

    private final SparkMax agitatorMotor;
    private final SparkMax lowerWheelMotor;
    private final SparkMax upperWheelMotor;

    private final RelativeEncoder agitatorEncoder;
    private final RelativeEncoder lowerWheelEncoder;
    private final RelativeEncoder upperWheelEncoder;

    private final SparkClosedLoopController lowerController;
    private final SparkClosedLoopController upperController;

    private final SparkMaxConfig lowerConfig;
    private final SparkMaxConfig upperConfig;

    public LauncherHardwareRev() {
        agitatorMotor = new SparkMax(AGITATOR_CAN_ID, MotorType.kBrushless);
        lowerWheelMotor = new SparkMax(LOWER_WHEEL_CAN_ID, MotorType.kBrushless);
        upperWheelMotor = new SparkMax(UPPER_WHEEL_CAN_ID, MotorType.kBrushless);

        agitatorEncoder = agitatorMotor.getEncoder();
        lowerWheelEncoder = lowerWheelMotor.getEncoder();
        upperWheelEncoder = upperWheelMotor.getEncoder();

        lowerController = lowerWheelMotor.getClosedLoopController();
        upperController = upperWheelMotor.getClosedLoopController();

        // agitator is open-loop only
        configureOpenLoop(agitatorMotor, agitatorInverted.getAsBoolean());

        // wheels use closed-loop velocity control
        lowerConfig = createWheelConfig(lowerWheelInverted.getAsBoolean());
        lowerWheelMotor.configure(lowerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        upperConfig = createWheelConfig(upperWheelInverted.getAsBoolean());
        upperWheelMotor.configure(upperConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    private void configureOpenLoop(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());
        config.openLoopRampRate(0.1);
        motor.configure(config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    private SparkMaxConfig createWheelConfig(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());
        config.closedLoopRampRate(0.1);

        // configure onboard velocity PID
        config.closedLoop.p(kP.getAsDouble());
        config.closedLoop.d(kD.getAsDouble());
        config.closedLoop.velocityFF(kV.getAsDouble());

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
    public void applyAgitatorVolts(double volts) {
        agitatorMotor.setVoltage(volts);
    }

    @Override
    public void resetPID(double kV, double kP, double kD) {
        lowerConfig.closedLoop.p(kP);
        lowerConfig.closedLoop.d(kD);
        lowerConfig.closedLoop.velocityFF(kV);
        lowerWheelMotor.configure(lowerConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        upperConfig.closedLoop.p(kP);
        upperConfig.closedLoop.d(kD);
        upperConfig.closedLoop.velocityFF(kV);
        upperWheelMotor.configure(upperConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public double getAgitatorRPM() {
        return agitatorEncoder.getVelocity();
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
    public double getAgitatorAmps() {
        return agitatorMotor.getOutputCurrent();
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

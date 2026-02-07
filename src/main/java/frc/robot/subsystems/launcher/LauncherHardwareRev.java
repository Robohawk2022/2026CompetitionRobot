package frc.robot.subsystems.launcher;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Config.Launcher.*;

/**
 * Implements {@link LauncherHardware} using REV SparkMax motors (NEO).
 */
public class LauncherHardwareRev implements LauncherHardware {

    private final SparkMax intakeMotor;
    private final SparkMax agitatorMotor;
    private final SparkMax lowerWheelMotor;
    private final SparkMax upperWheelMotor;

    private final RelativeEncoder intakeEncoder;
    private final RelativeEncoder agitatorEncoder;
    private final RelativeEncoder lowerWheelEncoder;
    private final RelativeEncoder upperWheelEncoder;

    public LauncherHardwareRev() {
        intakeMotor = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushless);
        agitatorMotor = new SparkMax(AGITATOR_CAN_ID, MotorType.kBrushless);
        lowerWheelMotor = new SparkMax(LOWER_WHEEL_CAN_ID, MotorType.kBrushless);
        upperWheelMotor = new SparkMax(UPPER_WHEEL_CAN_ID, MotorType.kBrushless);

        intakeEncoder = intakeMotor.getEncoder();
        agitatorEncoder = agitatorMotor.getEncoder();
        lowerWheelEncoder = lowerWheelMotor.getEncoder();
        upperWheelEncoder = upperWheelMotor.getEncoder();

        configureMotor(intakeMotor, intakeInverted.getAsBoolean(), IdleMode.kCoast);
        configureMotor(agitatorMotor, agitatorInverted.getAsBoolean(), IdleMode.kCoast);
        configureMotor(lowerWheelMotor, lowerWheelInverted.getAsBoolean(), IdleMode.kCoast);
        configureMotor(upperWheelMotor, upperWheelInverted.getAsBoolean(), IdleMode.kCoast);
    }

    private void configureMotor(SparkMax motor, boolean inverted, IdleMode idleMode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        config.inverted(inverted);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());
        config.openLoopRampRate(0.1);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void applyIntakeVolts(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void applyAgitatorVolts(double volts) {
        agitatorMotor.setVoltage(volts);
    }

    @Override
    public void applyLowerWheelVolts(double volts) {
        lowerWheelMotor.setVoltage(volts);
    }

    @Override
    public void applyUpperWheelVolts(double volts) {
        upperWheelMotor.setVoltage(volts);
    }

    @Override
    public double getIntakeRPM() {
        return intakeEncoder.getVelocity();
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
    public double getIntakeAmps() {
        return intakeMotor.getOutputCurrent();
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

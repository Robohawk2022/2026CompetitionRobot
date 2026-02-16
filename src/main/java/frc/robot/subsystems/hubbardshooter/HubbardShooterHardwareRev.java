package frc.robot.subsystems.hubbardshooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Config.HubbardShooter.*;

/**
 * Implements {@link HubbardShooterHardware} using REV SparkMax motors (NEO).
 * <p>
 * Two NEO motors with 12:1 gear reduction (3:1 * 4:1).
 * Open-loop voltage control only.
 */
public class HubbardShooterHardwareRev implements HubbardShooterHardware {

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;

    public HubbardShooterHardwareRev() {
        motor1 = new SparkMax(MOTOR_1_CAN_ID, MotorType.kBrushless);
        motor2 = new SparkMax(MOTOR_2_CAN_ID, MotorType.kBrushless);

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        configureMotor(motor1, motor1Inverted.getAsBoolean());
        configureMotor(motor2, motor2Inverted.getAsBoolean());
    }

    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());
        config.openLoopRampRate(0.1);
        motor.configure(config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void applyVoltage(double volts1, double volts2) {
        motor1.setVoltage(volts1);
        motor2.setVoltage(volts2);
    }

    @Override
    public double getMotor1RPM() {
        return encoder1.getVelocity();
    }

    @Override
    public double getMotor2RPM() {
        return encoder2.getVelocity();
    }

    @Override
    public double getMotor1Amps() {
        return motor1.getOutputCurrent();
    }

    @Override
    public double getMotor2Amps() {
        return motor2.getOutputCurrent();
    }
}

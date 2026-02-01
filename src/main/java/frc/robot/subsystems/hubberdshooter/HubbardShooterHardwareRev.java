package frc.robot.subsystems.hubberdshooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Config.HubberdShooter.*;

/**
 * Implements {@link HubberdShooterHardware} using REV SparkMax motors (NEO).
 */
public class HubbardShooterHardwareRev implements HubberdShooterHardware {

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;

    /**
     * Creates the REV hardware implementation using CAN IDs from Config.
     */
    public HubbardShooterHardwareRev() {
        motor1 = new SparkMax(MOTOR_1_CAN_ID, MotorType.kBrushless);
        motor2 = new SparkMax(MOTOR_2_CAN_ID, MotorType.kBrushless);

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        SparkMaxConfig config1 = new SparkMaxConfig();
        SparkMaxConfig config2 = new SparkMaxConfig();

        configureMotor(motor1, config1, motor1Inverted.getAsBoolean());
        configureMotor(motor2, config2, motor2Inverted.getAsBoolean());
    }

    private void configureMotor(SparkMax motor, SparkMaxConfig config, boolean inverted) {
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted);
        config.smartCurrentLimit((int) supplyCurrentLimit.getAsDouble());
        config.openLoopRampRate(0.1);
        config.closedLoopRampRate(0.1);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void applyVoltage(double volts1, double volts2) {
        motor1.setVoltage(volts1);
        motor2.setVoltage(volts2);
    }

    @Override
    public double getMotor1VelocityRPM() {
        // SparkMax encoder.getVelocity() returns RPM by default
        return encoder1.getVelocity();
    }

    @Override
    public double getMotor2VelocityRPM() {
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

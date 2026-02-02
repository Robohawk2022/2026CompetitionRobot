package frc.robot.subsystems.hubberdshooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Config.HubberdShooter.*;

/**
 * Implements {@link HubberdShooterHardware} using CTRE TalonFX motors (Falcon 500).
 */
public class HubberdShooterHardwareCTRE implements HubberdShooterHardware {

    private final TalonFX motor1;
    private final TalonFX motor2;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    /**
     * Creates the CTRE hardware implementation using CAN IDs from Config.
     */
    public HubberdShooterHardwareCTRE() {
        motor1 = new TalonFX(MOTOR_1_CAN_ID);
        motor2 = new TalonFX(MOTOR_2_CAN_ID);

        configureMotor(motor1, motor1Inverted.getAsBoolean());
        configureMotor(motor2, motor2Inverted.getAsBoolean());
    }

    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // current limits
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.getAsDouble();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.getAsDouble();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void applyVoltage(double volts1, double volts2) {
        motor1.setControl(voltageRequest.withOutput(volts1));
        motor2.setControl(voltageRequest.withOutput(volts2));
    }

    @Override
    public double getMotor1VelocityRPM() {
        // TalonFX getVelocity() returns rotations per second
        return motor1.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public double getMotor2VelocityRPM() {
        return motor2.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public double getMotor1Amps() {
        return motor1.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getMotor2Amps() {
        return motor2.getStatorCurrent().getValueAsDouble();
    }
}

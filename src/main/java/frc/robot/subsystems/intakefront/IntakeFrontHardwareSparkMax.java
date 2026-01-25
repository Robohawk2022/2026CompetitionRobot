package frc.robot.subsystems.intakefront;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Config.IntakeFront.*;

/**
 * Implements {@link IntakeFrontHardware} using a REV SparkMax motor controller.
 */
public class IntakeFrontHardwareSparkMax implements IntakeFrontHardware {

    private static final double MAX_VOLTAGE = 12.0;

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    /**
     * Creates the intake hardware on the configured CAN ID.
     */
    @SuppressWarnings("deprecation")
    public IntakeFrontHardwareSparkMax() {
        motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void applyVolts(double volts) {
        // Clamp voltage for safety
        volts = MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE);
        motor.setVoltage(volts);
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getMotorAmps() {
        return motor.getOutputCurrent();
    }
}

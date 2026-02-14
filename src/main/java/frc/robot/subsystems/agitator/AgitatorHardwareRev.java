package frc.robot.subsystems.agitator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Config.Agitator.*;

/**
 * Implements {@link AgitatorHardware} using a REV SparkMax motor (NEO).
 */
public class AgitatorHardwareRev implements AgitatorHardware {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    public AgitatorHardwareRev() {
        motor = new SparkMax(CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted.getAsBoolean());
        config.smartCurrentLimit(80);
        config.openLoopRampRate(0.0);
        motor.configure(config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void resetConfig() {
        config.inverted(inverted.getAsBoolean());
        motor.configure(config,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void applyVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public double getRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getAmps() {
        return motor.getOutputCurrent();
    }
}

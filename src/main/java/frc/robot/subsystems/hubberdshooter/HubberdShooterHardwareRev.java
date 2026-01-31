package frc.robot.subsystems.hubberdshooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Config.HubberdShooter.*;

/**
 * Implements {@link HubberdShooterHardware} using REV SparkMax motors (NEO).
 */
public class HubberdShooterHardwareRev implements HubberdShooterHardware {

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;
    private final SparkClosedLoopController closedLoopController1;
    private final SparkClosedLoopController closedLoopController2;
    private final SparkMaxConfig config1;
    private final SparkMaxConfig config2;

    /**
     * Creates the REV hardware implementation using CAN IDs from Config.
     */
    public HubberdShooterHardwareRev() {
        motor1 = new SparkMax(MOTOR_1_CAN_ID, MotorType.kBrushless);
        motor2 = new SparkMax(MOTOR_2_CAN_ID, MotorType.kBrushless);

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        closedLoopController1 = motor1.getClosedLoopController();
        closedLoopController2 = motor2.getClosedLoopController();

        config1 = new SparkMaxConfig();
        config2 = new SparkMaxConfig();

        configureMotor(motor1, config1, motor1Inverted.getAsBoolean());
        configureMotor(motor2, config2, motor2Inverted.getAsBoolean());
    }

    private void configureMotor(SparkMax motor, SparkMaxConfig config, boolean inverted) {
        config.idleMode(IdleMode.kCoast);
        config.inverted(inverted);

        // SparkMax uses smart current limit (supply-side limiting)
        config.smartCurrentLimit((int) supplyCurrentLimit.getAsDouble());

        // Configure closed-loop control for velocity mode
        // kV is in volts per rev/sec, but SparkMax velocityFF expects V/RPM, so divide by 60
        config.closedLoop.p(kP.getAsDouble());
        config.closedLoop.velocityFF(kV.getAsDouble() / 60.0);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void applyVoltage(double volts1, double volts2) {
        motor1.setVoltage(volts1);
        motor2.setVoltage(volts2);
    }

    @Override
    public void setVelocity(double rps1, double rps2) {
        // Convert RPS to RPM for SparkMax
        double rpm1 = rps1 * 60.0;
        double rpm2 = rps2 * 60.0;
        closedLoopController1.setReference(rpm1, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        closedLoopController2.setReference(rpm2, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void configurePid(double kV, double kP, double kS) {
        // Update closed-loop gains on both motors
        // Note: SparkMax doesn't have a direct kS equivalent in closed-loop config
        // kV is in volts per rev/sec, convert to V/RPM for SparkMax
        config1.closedLoop.p(kP);
        config1.closedLoop.velocityFF(kV / 60.0);
        motor1.configure(config1, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        config2.closedLoop.p(kP);
        config2.closedLoop.velocityFF(kV / 60.0);
        motor2.configure(config2, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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

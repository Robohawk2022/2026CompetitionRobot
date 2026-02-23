package frc.robot.subsystems.intakefront;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Config.IntakeFront.*;

/**
 * Implements {@link IntakeFrontHardware} using a REV SparkMax motor controller
 * with closed-loop velocity control.
 */
public class IntakeFrontHardwareSparkMax implements IntakeFrontHardware {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final SparkMaxConfig config;

    /**
     * Creates the intake hardware on the configured CAN ID.
     */
    public IntakeFrontHardwareSparkMax() {
        motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        // Configure motor
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());

        // Configure closed-loop control (primary encoder is default)
        config.closedLoop.p(kP.getAsDouble());
        config.closedLoop.velocityFF(kV.getAsDouble() / 60.0); // Convert from V/(rev/s) to V/RPM

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void setSpeed(double speedRPS) {
        // Convert RPS to RPM for the SparkMax
        double speedRPM = speedRPS * 60.0;
        closedLoopController.setReference(speedRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void resetPid(double kV, double kP) {
        // Update closed-loop gains
        config.closedLoop.p(kP);
        config.closedLoop.velocityFF(kV / 60.0); // Convert from V/(rev/s) to V/RPM
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getMotorAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}

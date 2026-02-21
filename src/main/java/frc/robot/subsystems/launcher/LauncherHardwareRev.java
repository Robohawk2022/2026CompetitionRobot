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
 * Implements {@link LauncherHardware} using 3 REV SparkMax motors (NEO)
 * with onboard closed-loop velocity control.
 * <p>
 * Feeder Left (CAN 1), Feeder Right (CAN 2), Shooter (CAN 3).
 */
public class LauncherHardwareRev implements LauncherHardware {

    private final SparkMax feederLeftMotor;
    private final SparkMax feederRightMotor;
    private final SparkMax shooterMotor;
    private final SparkMax shooterIntakeMotor;

    private final RelativeEncoder feederLeftEncoder;
    private final RelativeEncoder feederRightEncoder;
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder shooterIntakeEncoder;

    private final SparkClosedLoopController feederLeftController;
    private final SparkClosedLoopController feederRightController;
    private final SparkClosedLoopController shooterController;
    private final SparkClosedLoopController shooterIntakeController;

    private final SparkMaxConfig feederLeftConfig;
    private final SparkMaxConfig feederRightConfig;
    private final SparkMaxConfig shooterConfig;
    private final SparkMaxConfig shooterIntakeConfig;

    public LauncherHardwareRev() {
        feederLeftMotor = new SparkMax(FEEDER_LEFT_CAN_ID, MotorType.kBrushless);
        feederRightMotor = new SparkMax(FEEDER_RIGHT_CAN_ID, MotorType.kBrushless);
        shooterMotor = new SparkMax(SHOOTER_CAN_ID, MotorType.kBrushless);
        shooterIntakeMotor = new SparkMax(SHOOTER_INTAKE_CAN_ID, MotorType.kBrushless);

        feederLeftEncoder = feederLeftMotor.getEncoder();
        feederRightEncoder = feederRightMotor.getEncoder();
        shooterEncoder = shooterMotor.getEncoder();
        shooterIntakeEncoder = shooterIntakeMotor.getEncoder();

        feederLeftController = feederLeftMotor.getClosedLoopController();
        feederRightController = feederRightMotor.getClosedLoopController();
        shooterController = shooterMotor.getClosedLoopController();
        shooterIntakeController = shooterIntakeMotor.getClosedLoopController();

        // feeder motors get independent PID gains
        feederLeftConfig = createMotorConfig(FEEDER_LEFT_INVERTED,
                feederLeftKV.getAsDouble(), feederLeftKP.getAsDouble());
        feederLeftMotor.configure(feederLeftConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        feederRightConfig = createMotorConfig(FEEDER_RIGHT_INVERTED,
                feederRightKV.getAsDouble(), feederRightKP.getAsDouble());
        feederRightMotor.configure(feederRightConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // shooter has its own PID gains
        shooterConfig = createMotorConfig(SHOOTER_INVERTED,
                shooterKV.getAsDouble(), shooterKP.getAsDouble());
        shooterMotor.configure(shooterConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // shooter intake - same side as shooter, feeds into it
        shooterIntakeConfig = createMotorConfig(SHOOTER_INTAKE_INVERTED,
                shooterIntakeKV.getAsDouble(), shooterIntakeKP.getAsDouble());
        shooterIntakeMotor.configure(shooterIntakeConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    private SparkMaxConfig createMotorConfig(boolean inverted, double kV, double kP) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(inverted);
        config.smartCurrentLimit((int) currentLimit.getAsDouble());
        config.closedLoopRampRate(0.1);

        config.closedLoop.p(kP);
        config.closedLoop.feedForward.kV(kV);

        return config;
    }

    private void setMotorRPM(SparkMax motor, SparkClosedLoopController controller, double rpm) {
        if (rpm == 0) {
            motor.set(0);
        } else {
            controller.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    @Override
    public void setFeederLeftRPM(double rpm) {
        setMotorRPM(feederLeftMotor, feederLeftController, rpm);
    }

    @Override
    public void setFeederRightRPM(double rpm) {
        setMotorRPM(feederRightMotor, feederRightController, rpm);
    }

    @Override
    public void setShooterRPM(double rpm) {
        setMotorRPM(shooterMotor, shooterController, rpm);
    }

    @Override
    public double getFeederLeftRPM() {
        return feederLeftEncoder.getVelocity();
    }

    @Override
    public double getFeederRightRPM() {
        return feederRightEncoder.getVelocity();
    }

    @Override
    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    @Override
    public double getFeederLeftAmps() {
        return feederLeftMotor.getOutputCurrent();
    }

    @Override
    public double getFeederRightAmps() {
        return feederRightMotor.getOutputCurrent();
    }

    @Override
    public double getShooterAmps() {
        return shooterMotor.getOutputCurrent();
    }

    @Override
    public void resetFeederLeftPID(double kV, double kP, double kI, double kD) {
        applyPID(feederLeftConfig, feederLeftMotor, FEEDER_LEFT_INVERTED, kV, kP, kI, kD);
    }

    @Override
    public void resetFeederRightPID(double kV, double kP, double kI, double kD) {
        applyPID(feederRightConfig, feederRightMotor, FEEDER_RIGHT_INVERTED, kV, kP, kI, kD);
    }

    @Override
    public void resetShooterPID(double kV, double kP, double kI, double kD) {
        applyPID(shooterConfig, shooterMotor, SHOOTER_INVERTED, kV, kP, kI, kD);
    }

    @Override
    public void setShooterIntakeRPM(double rpm) {
        setMotorRPM(shooterIntakeMotor, shooterIntakeController, rpm);
    }

    @Override
    public double getShooterIntakeRPM() {
        return shooterIntakeEncoder.getVelocity();
    }

    @Override
    public double getShooterIntakeAmps() {
        return shooterIntakeMotor.getOutputCurrent();
    }

    @Override
    public void resetShooterIntakePID(double kV, double kP, double kI, double kD) {
        applyPID(shooterIntakeConfig, shooterIntakeMotor, SHOOTER_INTAKE_INVERTED, kV, kP, kI, kD);
    }

    private void applyPID(SparkMaxConfig config, SparkMax motor, boolean inverted,
                           double kV, double kP, double kI, double kD) {
        config.inverted(inverted);
        config.closedLoop.p(kP);
        config.closedLoop.i(kI);
        config.closedLoop.d(kD);
        config.closedLoop.feedForward.kV(kV);
        motor.configure(config,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }
}

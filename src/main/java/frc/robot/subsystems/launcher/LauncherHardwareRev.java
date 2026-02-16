package frc.robot.subsystems.launcher;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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

    public static final boolean FEEDER_LEFT_INVERTED = true;
    public static final boolean FEEDER_RIGHT_INVERTED = false;
    public static final boolean SHOOTER_INVERTED = false;

    public static final int CURRENT_LIMIT = 60;

    private final SparkMax feederLeftMotor;
    private final SparkMax feederRightMotor;
    private final SparkMax shooterMotor;

    private final RelativeEncoder feederLeftEncoder;
    private final RelativeEncoder feederRightEncoder;
    private final RelativeEncoder shooterEncoder;

    private final SparkClosedLoopController feederLeftController;
    private final SparkClosedLoopController feederRightController;
    private final SparkClosedLoopController shooterController;

    private final SparkMaxConfig feederLeftConfig;
    private final SparkMaxConfig feederRightConfig;
    private final SparkMaxConfig shooterConfig;

    public LauncherHardwareRev(int feederLeftCanId, int feederRightCanId, int shooterCanId) {
        feederLeftMotor = new SparkMax(feederLeftCanId, MotorType.kBrushless);
        feederRightMotor = new SparkMax(feederRightCanId, MotorType.kBrushless);
        shooterMotor = new SparkMax(shooterCanId, MotorType.kBrushless);

        feederLeftEncoder = feederLeftMotor.getEncoder();
        feederRightEncoder = feederRightMotor.getEncoder();
        shooterEncoder = shooterMotor.getEncoder();

        feederLeftController = feederLeftMotor.getClosedLoopController();
        feederRightController = feederRightMotor.getClosedLoopController();
        shooterController = shooterMotor.getClosedLoopController();

        // feeder motors get independent PID gains
        feederLeftConfig = createMotorConfig(FEEDER_LEFT_INVERTED,
                feederLeftKV.getAsDouble(), feederLeftKP.getAsDouble());
        feederLeftMotor.configure(feederLeftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        feederRightConfig = createMotorConfig(FEEDER_RIGHT_INVERTED,
                feederRightKV.getAsDouble(), feederRightKP.getAsDouble());
        feederRightMotor.configure(feederRightConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // shooter has its own PID gains
        shooterConfig = createMotorConfig(SHOOTER_INVERTED,
                shooterKV.getAsDouble(), shooterKP.getAsDouble());
        shooterMotor.configure(shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    private SparkMaxConfig createMotorConfig(boolean inverted, double kV, double kP) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(inverted);
        config.smartCurrentLimit(CURRENT_LIMIT);
        config.closedLoopRampRate(0.1);

        config.closedLoop.p(kP);
        config.closedLoop.feedForward.kV(kV);

        return config;
    }

    private void setMotorRPM(SparkMax motor, SparkClosedLoopController controller, double rpm) {
        if (rpm == 0) {
            motor.set(0);
        } else {
            controller.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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

    private void applyPID(SparkMaxConfig config, SparkMax motor, boolean inverted,
                           double kV, double kP, double kI, double kD) {
        config.inverted(inverted);
        config.closedLoop.p(kP);
        config.closedLoop.i(kI);
        config.closedLoop.d(kD);
        config.closedLoop.feedForward.kV(kV);
        motor.configure(config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }
}

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Config.PIDFConfig;
import frc.robot.util.Util;

import static frc.robot.Config.Shooter.shooterPid;

/**
 * Implementation of {@link ShooterHardware} on a SparkMax motor controller.
 */
public class ShooterHardwareRev implements ShooterHardware {

    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;

//region State & constructor ---------------------------------------------------

    final SparkMax shooterMotor;
    final SparkMaxConfig shooterConfig;
    final RelativeEncoder shooterEncoder;
    final SparkClosedLoopController shooterController;

    public ShooterHardwareRev(int shooterId) {
        shooterConfig = new SparkMaxConfig();
        shooterMotor = createMotor(shooterId, shooterConfig);
        shooterEncoder = shooterMotor.getEncoder();
        shooterController = shooterMotor.getClosedLoopController();
    }

    /**
     * @return a new motor with the supplied CAN ID and default settings
     */
    private SparkMax createMotor(int canId, SparkMaxConfig config) {
        SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
        config.smartCurrentLimit(CURRENT_LIMIT);
        config.inverted(INVERTED);
        config.idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

//endregion

//region Implementation --------------------------------------------------------

    @Override
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

    @Override
    public void resetPid() {
        resetPid(shooterMotor, shooterConfig, shooterPid);
        Util.log("[shooter] reset PID values");
    }

    private void resetPid(SparkMax motor, SparkMaxConfig config, PIDFConfig pidf) {
        config.closedLoop.p(pidf.p.getAsDouble());
        config.closedLoop.i(pidf.i.getAsDouble());
        config.closedLoop.iZone(pidf.iz.getAsDouble());
        config.closedLoop.d(pidf.d.getAsDouble());
        config.closedLoop.feedForward.kV(pidf.v.getAsDouble());
        motor.configure(config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void applyRpm(double shooterRpm) {
        if (shooterRpm == 0.0) {
            shooterController.setSetpoint(0.0, ControlType.kVoltage);
        } else {
            shooterController.setSetpoint(shooterRpm, ControlType.kVelocity);
        }
    }

//endregion

}

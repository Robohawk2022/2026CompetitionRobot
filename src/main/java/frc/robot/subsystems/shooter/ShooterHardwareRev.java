package frc.robot.subsystems.shooter;

import java.util.HashMap;
import java.util.Map;

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

    /** last PID gains pushed to each SPARK, so we only reconfigure on change */
    final Map<SparkMax, double[]> lastAppliedPid = new HashMap<>();

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
        // ramp so spin-up doesn't slam all SPARKs to their current limit at once
        config.openLoopRampRate(0.5);
        config.closedLoopRampRate(0.5);
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
    public double getShooterAmps() {
        return shooterMotor.getOutputCurrent();
    }

    @Override
    public void resetPid() {
        resetPid(shooterMotor, shooterConfig, shooterPid);
    }

    /**
     * Pushes PID gains to the SPARK only if they changed since the last
     * push. SparkMax.configure() is a blocking CAN transaction, and this
     * used to run on every command start (including the default coast
     * command, i.e. every trigger release), stalling the main loop exactly
     * when the driver was shooting or intaking.
     */
    private void resetPid(SparkMax motor, SparkMaxConfig config, PIDFConfig pidf) {
        double p = pidf.p.getAsDouble();
        double i = pidf.i.getAsDouble();
        double iz = pidf.iz.getAsDouble();
        double d = pidf.d.getAsDouble();
        double v = pidf.v.getAsDouble();

        double[] last = lastAppliedPid.get(motor);
        if (last != null
                && last[0] == p && last[1] == i && last[2] == iz
                && last[3] == d && last[4] == v) {
            return;
        }

        config.closedLoop.p(p);
        config.closedLoop.i(i);
        config.closedLoop.iZone(iz);
        config.closedLoop.d(d);
        config.closedLoop.feedForward.kV(v);
        motor.configure(config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        lastAppliedPid.put(motor, new double[] {p, i, iz, d, v});
        Util.log("[shooter] applied PID gains to CAN %d", motor.getDeviceId());
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

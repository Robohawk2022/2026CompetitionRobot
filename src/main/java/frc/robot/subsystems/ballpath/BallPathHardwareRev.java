package frc.robot.subsystems.ballpath;

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

import static frc.robot.Config.BallPath.agitatorPid;
import static frc.robot.Config.BallPath.feederPid;
import static frc.robot.Config.BallPath.intakePid;

/**
 * Implementation of {@link BallPathHardware} on SparkMax motor controllers.
 */
public class BallPathHardwareRev implements BallPathHardware {

    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;

//region State & constructor ---------------------------------------------------

    final SparkMax intakeMotor;
    final SparkMax feederMotor;
    final SparkMax agitatorMotor;

    final SparkMaxConfig intakeConfig;
    final SparkMaxConfig feederConfig;
    final SparkMaxConfig agitatorConfig;

    final RelativeEncoder intakeEncoder;
    final RelativeEncoder feederEncoder;
    final RelativeEncoder agitatorEncoder;

    final SparkClosedLoopController intakeController;
    final SparkClosedLoopController feederController;
    final SparkClosedLoopController agitatorController;

    /** last PID gains pushed to each SPARK, so we only reconfigure on change */
    final Map<SparkMax, double[]> lastAppliedPid = new HashMap<>();

    public BallPathHardwareRev(int intakeId, int feederId, int agitatorId) {

        // create configurations
        intakeConfig = new SparkMaxConfig();
        feederConfig = new SparkMaxConfig();
        agitatorConfig = new SparkMaxConfig();

        // create yon motors
        // the intake motor generally wants to spin the opposite direction
        // of the others, so we will invert him here
        intakeMotor = createMotor(intakeId, intakeConfig, !INVERTED);
        feederMotor = createMotor(feederId, feederConfig, INVERTED);
        agitatorMotor = createMotor(agitatorId, agitatorConfig, INVERTED);

        // grab the encoders
        intakeEncoder = intakeMotor.getEncoder();
        feederEncoder = feederMotor.getEncoder();
        agitatorEncoder = agitatorMotor.getEncoder();

        // grab the PID controllers
        intakeController = intakeMotor.getClosedLoopController();
        feederController = feederMotor.getClosedLoopController();
        agitatorController = agitatorMotor.getClosedLoopController();
    }

    /**
     * @return a new motor with the supplied CAN ID and default settings
     */
    private SparkMax createMotor(int canId, SparkMaxConfig config, boolean inverted) {
        SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
        config.smartCurrentLimit(CURRENT_LIMIT);
        // ramp so spin-up doesn't slam all SPARKs to their current limit at once
        config.openLoopRampRate(0.25);
        config.closedLoopRampRate(0.25);
        config.inverted(inverted);
        config.idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

//endregion

//region Getters ---------------------------------------------------------------

    @Override
    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public double getFeederVelocity() {
        return feederEncoder.getVelocity();
    }

    @Override
    public double getAgitatorVelocity() {
        return agitatorEncoder.getVelocity();
    }

    @Override
    public double getIntakeAmps() {
        return intakeMotor.getOutputCurrent();
    }

    @Override
    public double getFeederAmps() {
        return feederMotor.getOutputCurrent();
    }

    @Override
    public double getAgitatorAmps() {
        return agitatorMotor.getOutputCurrent();
    }

//endregion

//region PID configuration -----------------------------------------------------

    @Override
    public void resetPid() {
        resetPid(intakeMotor, intakeConfig, intakePid);
        resetPid(feederMotor, feederConfig, feederPid);
        resetPid(agitatorMotor, agitatorConfig, agitatorPid);
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
        Util.log("[ballpath] applied PID gains to CAN %d", motor.getDeviceId());
    }

//endregion

//region Applying RPM ----------------------------------------------------------

    @Override
    public void applyRpm(double intakeRpm, double feederRpm, double agitatorRpm) {
        applyRpm(intakeController, intakeRpm);
        applyRpm(feederController, feederRpm);
        applyRpm(agitatorController, agitatorRpm);
    }

    private void applyRpm(SparkClosedLoopController pid, double rpm) {
        if (rpm == 0.0) {
            pid.setSetpoint(0.0, ControlType.kVoltage);
        } else {
            pid.setSetpoint(rpm, ControlType.kVelocity);
        }
    }

//endregion

}

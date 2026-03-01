package frc.robot.subsystems.ballpath;

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

//endregion

//region PID configuration -----------------------------------------------------

    @Override
    public void resetPid() {
        resetPid(intakeMotor, intakeConfig, intakePid);
        resetPid(feederMotor, feederConfig, feederPid);
        resetPid(agitatorMotor, agitatorConfig, agitatorPid);
        Util.log("[ballpath] reset PID values");
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

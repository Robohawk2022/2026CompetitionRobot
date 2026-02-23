package frc.robot.subsystems.launcher;

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

import java.util.function.Consumer;

import static frc.robot.Config.Launcher.agitatorPid;
import static frc.robot.Config.Launcher.feederPid;
import static frc.robot.Config.Launcher.shooterPid;
import static frc.robot.Config.Launcher.intakePid;

/**
 * Implementation of {@link LauncherHardware} on SparkMax motor controllers
 */
public class LauncherHardwareRev implements LauncherHardware{

    public static final int CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;

//region State & constructor ---------------------------------------------------

    final SparkMax intakeMotor;
    final SparkMax feederMotor;
    final SparkMax agitatorMotor;
    final SparkMax shooterMotor;

    final RelativeEncoder intakeEncoder;
    final RelativeEncoder feederEncoder;
    final RelativeEncoder agitatorEncoder;
    final RelativeEncoder shooterEncoder;

    final SparkClosedLoopController intakeController;
    final SparkClosedLoopController feederController;
    final SparkClosedLoopController agitatorController;
    final SparkClosedLoopController shooterController;

    public LauncherHardwareRev(int intakeId, int feederId, int agitatorId, int shooterId) {

        // create yon motors
        intakeMotor = createMotor(intakeId);
        feederMotor = createMotor(feederId);
        agitatorMotor = createMotor(agitatorId);
        shooterMotor = createMotor(shooterId);

        // grab the encoders
        intakeEncoder = intakeMotor.getEncoder();
        feederEncoder = feederMotor.getEncoder();
        agitatorEncoder = agitatorMotor.getEncoder();
        shooterEncoder = shooterMotor.getEncoder();

        // grab the PID controllers
        intakeController = intakeMotor.getClosedLoopController();
        feederController = feederMotor.getClosedLoopController();
        agitatorController = agitatorMotor.getClosedLoopController();
        shooterController = shooterMotor.getClosedLoopController();
    }

    /**
     * @return a new motor with the supplied CAN ID and default settings
     */
    private SparkMax createMotor(int canId) {
        SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
        configureMotor(motor, config -> {
            config.smartCurrentLimit(CURRENT_LIMIT);
            config.inverted(INVERTED);
            config.idleMode(IdleMode.kCoast);
        });
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
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

//endregion

//region PID configuration -----------------------------------------------------

    @Override
    public void resetPid() {
        resetPid(intakeMotor, intakePid);
        resetPid(feederMotor, feederPid);
        resetPid(agitatorMotor, agitatorPid);
        resetPid(shooterMotor, shooterPid);
        Util.log("[launcher] reset PID values");
    }

    /**
     * Applies closed-loop configuration to the supplied motor
     */
    private void resetPid(SparkMax motor, PIDFConfig pidf) {
        configureMotor(motor, config -> {
            config.closedLoop.p(pidf.p.getAsDouble());
            config.closedLoop.i(pidf.i.getAsDouble());
            config.closedLoop.iZone(pidf.iz.getAsDouble());
            config.closedLoop.d(pidf.d.getAsDouble());
            config.closedLoop.feedForward.kV(pidf.v.getAsDouble());
        });
    }

//endregion

//region Applying RPM ----------------------------------------------------------

    @Override
    public void applyRpm(double intakeRpm, double feederRpm, double agitatorRpm, double shooterRpm) {
        applyRpm(intakeController, intakeRpm);
        applyRpm(feederController, feederRpm);
        applyRpm(agitatorController, agitatorRpm);
        applyRpm(shooterController, shooterRpm);
    }

    /**
     * Applies appropriate voltage or speed to the supplied controller
     */
    private void applyRpm(SparkClosedLoopController pid, double rpm) {
        if (rpm == 0.0) {
            pid.setSetpoint(0.0, ControlType.kVoltage);
        } else {
            pid.setSetpoint(rpm, ControlType.kVelocity);
        }
    }

//endregion

//region Helper ----------------------------------------------------------------

    /**
     * Updates motor configuration, using the supplied callback to update it
     * prior to saving
     */
    private void configureMotor(SparkMax motor, Consumer<SparkMaxConfig> consumer) {
        SparkMaxConfig config = new SparkMaxConfig();
        consumer.accept(config);
        motor.configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

//endregion

}

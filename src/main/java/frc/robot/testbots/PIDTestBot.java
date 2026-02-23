package frc.robot.testbots;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * PID testbot. To use this:
 * <ul>
 *
 *     <li>Ensure that CAN IDs identify front/back correctly</li>
 *
 * </ul>
 */
public class PIDTestBot extends TimedRobot {

    public static final int FRONT_CAN = 8;
    public static final int BACK_CAN = 9;
    public static final int CURRENT_LIMIT = 60;

    public enum Mode {
        FRONT_REVERSE,
        FRONT_FORWARD,
        BACK_REVERSE,
        BACK_FORWARD,
        INTAKE,
        EJECT,
        SHOOT,
        IDLE
    }

    final SparkMax backMotor;
    final SparkMax frontMotor;
    final RelativeEncoder backEncoder;
    final RelativeEncoder frontEncoder;
    final SparkClosedLoopController backController;
    final SparkClosedLoopController frontController;
    double frontP = 0.0;
    double frontV = 0.002;
    double backP = 0.0;
    double backV = 0.002;
    double intakeVelocity = 500;
    double ejectVelocity = 500;
    double shootVelocity = 500;
    double testVelocity = 500;
    double backRpm;
    double frontRpm;
    double backAmps;
    double frontAmps;
    CommandXboxController controller;
    Mode mode;

    public PIDTestBot() {

        // create stuff
        backMotor = motor(BACK_CAN);
        backEncoder = backMotor.getEncoder();
        backController = backMotor.getClosedLoopController();
        frontMotor = motor(FRONT_CAN);
        frontEncoder = frontMotor.getEncoder();
        frontController = frontMotor.getClosedLoopController();
        controller = new CommandXboxController(0);
        mode = Mode.IDLE;

        // =================================================================
        // key bindings
        // =================================================================
        cmd(controller.leftTrigger(), Mode.BACK_REVERSE);
        cmd(controller.leftBumper(), Mode.BACK_FORWARD);
        cmd(controller.rightTrigger(), Mode.FRONT_REVERSE);
        cmd(controller.rightBumper(), Mode.FRONT_FORWARD);
        cmd(controller.a(), Mode.INTAKE);
        cmd(controller.b(), Mode.EJECT);
        cmd(controller.x(), Mode.SHOOT);

        // register stuff
        SmartDashboard.putData("PIDTestBot", builder -> {
            builder.addDoubleProperty("Back/Rpm", () -> backRpm, null);
            builder.addDoubleProperty("Back/Amps", () -> backAmps, null);
            builder.addDoubleProperty("Back/kP", () -> backP, val -> backP = val);
            builder.addDoubleProperty("Back/kV", () -> backV, val -> backV = val);
            builder.addDoubleProperty("Front/Rpm", () -> frontRpm, null);
            builder.addDoubleProperty("Front/Amps", () -> frontAmps, null);
            builder.addDoubleProperty("Front/kP", () -> frontP, val -> frontP = val);
            builder.addDoubleProperty("Front/kV", () -> frontV, val -> frontV = val);
            builder.addStringProperty("Mode", () -> mode.toString(), null);
            builder.addDoubleProperty("Velocity/Intake", () -> intakeVelocity, val -> intakeVelocity = val);
            builder.addDoubleProperty("Velocity/Shoot", () -> shootVelocity, val -> shootVelocity = val);
            builder.addDoubleProperty("Velocity/Eject", () -> ejectVelocity, val -> ejectVelocity = val);
            builder.addDoubleProperty("Velocity/Test", () -> testVelocity, val -> testVelocity = val);
        });
    }

    private void cmd(Trigger trigger, Mode mode) {

        Command log = Commands.print(mode.toString());
        Command setPd = Commands.runOnce(() -> {
            vp(frontMotor, frontV, frontP);
            vp(backMotor, backV, backP);
        });
        Command run = Commands.run(() -> this.mode = mode);

        trigger.whileTrue(log
                .andThen(setPd)
                .andThen(run)
                .finallyDo(() -> this.mode = Mode.IDLE));
    }

    /**
     * Sets the output for the two different motors - they're either coasting
     * or in velocity control mode
     */
    private void set(double back, double front) {
        if (back == 0.0) {
            backMotor.set(0.0);
        } else {
            backController.setSetpoint(back, ControlType.kVelocity);
        }
        if (front == 0.0) {
            frontMotor.set(0.0);
        } else {
            frontController.setSetpoint(front, ControlType.kVelocity);
        }
    }

    /**
     * Captures stats and runs commands
     */
    @Override
    public void robotPeriodic() {

        frontAmps = frontMotor.getOutputCurrent();
        backAmps = backMotor.getOutputCurrent();
        backRpm = backEncoder.getVelocity();
        frontRpm = frontEncoder.getVelocity();

        CommandScheduler.getInstance().run();

    }

    /**
     * Applies output to the motors as appropriate
     */
    @Override
    public void teleopPeriodic() {
        switch (mode) {
            case IDLE -> set(0.0, 0.0);

            // TODO check the signs on these
            case BACK_REVERSE -> set(-testVelocity, 0.0);
            case BACK_FORWARD -> set(testVelocity, 0.0);
            case FRONT_REVERSE -> set(0.0, -testVelocity);
            case FRONT_FORWARD -> set(0.0, testVelocity);
            case INTAKE -> set(intakeVelocity, intakeVelocity);
            case EJECT -> set(-ejectVelocity, -ejectVelocity);
            case SHOOT -> set(-shootVelocity, shootVelocity);

        }
    }

    /**
     * Set the V and P parameters for closed loop
     */
    private void vp(SparkMax motor, double v, double p) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(p);
        config.closedLoop.feedForward.kV(v);
        motor.configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * Creates a motor
     */
    private SparkMax motor(int canID) {
        SparkMax motor = new SparkMax(canID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(CURRENT_LIMIT);
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);
        config.openLoopRampRate(0.1);
        config.closedLoopRampRate(0.1);
        motor.configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        return motor;
    }
}

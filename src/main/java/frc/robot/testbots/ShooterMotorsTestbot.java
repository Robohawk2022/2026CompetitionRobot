package frc.robot.testbots;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameController;

import static frc.robot.util.Util.clampVolts;

/**
 * Testbot for three shooter motors: intake, index, and shoot.
 * <p>
 * Each motor is controlled by a SparkMax. Positive voltage = clockwise (CW),
 * negative voltage = counter-clockwise (CCW).
 * <p>
 * Button mappings:
 * <ul>
 *   <li>A - Intake function: intake 60% CCW, index 5% CW, shoot off</li>
 *   <li>X - Shoot function: intake off, index 60% CW, shoot 60% CCW</li>
 * </ul>
 * <p>
 * All percentages are configurable via SmartDashboard.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=ShooterMotorsTestbot}
 */
public class ShooterMotorsTestbot extends TimedRobot {

    // CAN IDs for the motors
    static final int INTAKE_CAN_ID = 20;
    static final int INDEX_CAN_ID = 21;
    static final int SHOOT_CAN_ID = 22;

    // max voltage for percentage calculations
    static final double MAX_VOLTS = 12.0;

    final SparkMax intakeMotor;
    final SparkMax indexMotor;
    final SparkMax shootMotor;
    final GameController controller;

    // configurable percentages (set via SmartDashboard)
    // intake function settings
    double intakeFuncIntakePct = 60.0;  // CCW
    double intakeFuncIndexPct = 5.0;    // CW

    // shoot function settings
    double shootFuncIndexPct = 60.0;    // CW
    double shootFuncShootPct = 60.0;    // CCW

    // track current state for dashboard
    String currentMode = "idle";
    double intakeVoltsCmd = 0;
    double indexVoltsCmd = 0;
    double shootVoltsCmd = 0;

    public ShooterMotorsTestbot() {
        // create motors
        intakeMotor = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushless);
        indexMotor = new SparkMax(INDEX_CAN_ID, MotorType.kBrushless);
        shootMotor = new SparkMax(SHOOT_CAN_ID, MotorType.kBrushless);

        // configure motors with coast mode
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shootMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = new GameController(0);

        // button bindings
        controller.a().whileTrue(intakeCommand());
        controller.x().whileTrue(shootCommand());

        // dashboard for configuration
        SmartDashboard.putData("ShooterMotorsTestbot", builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);

            // intake function percentages (editable)
            builder.addDoubleProperty("IntakeFunc/IntakePct", () -> intakeFuncIntakePct, val -> intakeFuncIntakePct = val);
            builder.addDoubleProperty("IntakeFunc/IndexPct", () -> intakeFuncIndexPct, val -> intakeFuncIndexPct = val);

            // shoot function percentages (editable)
            builder.addDoubleProperty("ShootFunc/IndexPct", () -> shootFuncIndexPct, val -> shootFuncIndexPct = val);
            builder.addDoubleProperty("ShootFunc/ShootPct", () -> shootFuncShootPct, val -> shootFuncShootPct = val);

            // commanded voltages (read-only)
            builder.addDoubleProperty("IntakeVolts", () -> intakeVoltsCmd, null);
            builder.addDoubleProperty("IndexVolts", () -> indexVoltsCmd, null);
            builder.addDoubleProperty("ShootVolts", () -> shootVoltsCmd, null);
        });

        System.out.println(">>> ShooterMotorsTestbot initialized");
        System.out.println(">>> A=intake function, X=shoot function");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // stop all motors when disabled
        intakeMotor.setVoltage(0);
        indexMotor.setVoltage(0);
        shootMotor.setVoltage(0);
        currentMode = "disabled";
    }

    //==========================================================================
    // Helper
    //==========================================================================

    /** Convert percentage (0-100) to voltage, positive = CW, negative = CCW */
    private double pctToVolts(double pct) {
        return clampVolts((pct / 100.0) * MAX_VOLTS);
    }

    //==========================================================================
    // Commands
    //==========================================================================

    /**
     * Intake function: intake 60% CCW, index 5% CW, shoot off.
     * @return a command that runs the intake function
     */
    private Command intakeCommand() {
        return Commands.startEnd(
            () -> {
                currentMode = "intake";
                // intake CCW = negative voltage
                intakeVoltsCmd = -pctToVolts(intakeFuncIntakePct);
                intakeMotor.setVoltage(intakeVoltsCmd);
                // index CW = positive voltage
                indexVoltsCmd = pctToVolts(intakeFuncIndexPct);
                indexMotor.setVoltage(indexVoltsCmd);
                // shoot off
                shootVoltsCmd = 0;
                shootMotor.setVoltage(0);
            },
            () -> {
                intakeVoltsCmd = 0;
                indexVoltsCmd = 0;
                shootVoltsCmd = 0;
                intakeMotor.setVoltage(0);
                indexMotor.setVoltage(0);
                shootMotor.setVoltage(0);
                currentMode = "idle";
            }
        );
    }

    /**
     * Shoot function: intake off, index 60% CW, shoot 60% CCW.
     * @return a command that runs the shoot function
     */
    private Command shootCommand() {
        return Commands.startEnd(
            () -> {
                currentMode = "shoot";
                // intake off
                intakeVoltsCmd = 0;
                intakeMotor.setVoltage(0);
                // index CW = positive voltage
                indexVoltsCmd = pctToVolts(shootFuncIndexPct);
                indexMotor.setVoltage(indexVoltsCmd);
                // shoot CCW = negative voltage
                shootVoltsCmd = -pctToVolts(shootFuncShootPct);
                shootMotor.setVoltage(shootVoltsCmd);
            },
            () -> {
                intakeVoltsCmd = 0;
                indexVoltsCmd = 0;
                shootVoltsCmd = 0;
                intakeMotor.setVoltage(0);
                indexMotor.setVoltage(0);
                shootMotor.setVoltage(0);
                currentMode = "idle";
            }
        );
    }
}

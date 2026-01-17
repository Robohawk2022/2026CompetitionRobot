package frc.robot.subsystems.shooter2;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class ShooterSubsystem2 extends SubsystemBase {

    final SparkMax motor1;
    final SparkMax motor2;
    final RelativeEncoder encoder1;
    final RelativeEncoder encoder2;
    final SparkClosedLoopController pid1;
    final SparkClosedLoopController pid2;
    double p1;
    double p2;
    double v1;
    double v2;
    String command;

    public ShooterSubsystem2(int id1, int id2) {

        motor1 = new SparkMax(id1, MotorType.kBrushless);
        motor2 = new SparkMax(id2, MotorType.kBrushless);
        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();
        pid1 = motor1.getClosedLoopController();
        pid2 = motor2.getClosedLoopController();

        SmartDashboard.putData("ShooterSubsystem", builder -> {
            builder.addDoubleProperty("Tuning/1_kP", () -> p1, val -> p1 = val);
            builder.addDoubleProperty("Tuning/1_kV", () -> v1, val -> v1 = val);
            builder.addDoubleProperty("Tuning/2_kP", () -> p2, val -> p2 = val);
            builder.addDoubleProperty("Tuning/2_kV", () -> v2, val -> v2 = val);
            builder.addDoubleProperty("Speed/Motor1", encoder1::getVelocity, null);
            builder.addDoubleProperty("Speed/Motor2", encoder2::getVelocity, null);
        });
    }

    public Command idleCommand() {
        return run(() -> {
            command = "idle";
            motor1.set(0.0);
            motor2.set(0.0);
        });
    }

    public Command teleopCommand(DoubleSupplier input) {
        return run(() -> {
            command = "teleop";
            double value = MathUtil.clamp(input.getAsDouble(), -1.0, 1.0);
            motor1.set(value);
            motor2.set(value);
        });
    }

    private void setPid(SparkMax motor, double p, double v) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(p);
        config.closedLoop.feedForward.kV(v);
        motor.configure(config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public Command runAt(double rps1, double rps2) {
        return startRun(
                () -> {
                    setPid(motor1, p1, v1);
                    setPid(motor2, p2, v2);
                },
                () -> {
                    command = "closed-loop";
                    pid1.setSetpoint(rps1, ControlType.kVelocity);
                    pid2.setSetpoint(rps2, ControlType.kVelocity);
                }
        );
    }
}

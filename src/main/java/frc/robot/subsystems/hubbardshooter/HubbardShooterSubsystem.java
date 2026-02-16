package frc.robot.subsystems.hubbardshooter;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.HubbardShooter.*;

/**
 * Subsystem for the HubbardShooter mechanism.
 * <p>
 * Two NEO motors (CAN 51, 52) with 12:1 gear reduction (3:1 * 4:1).
 * Open-loop voltage control with four operating modes:
 * <ul>
 *   <li><b>Off</b> - Motors stopped (0V)</li>
 *   <li><b>Intake</b> - Both motors same direction at configurable power</li>
 *   <li><b>Outtake</b> - Both motors reversed at configurable power</li>
 *   <li><b>Shooting</b> - Motors counter-rotate at configurable power</li>
 * </ul>
 */
public class HubbardShooterSubsystem extends SubsystemBase {

    static final boolean verboseLogging = true;

//region Implementation --------------------------------------------------------

    final HubbardShooterHardware hardware;
    String currentMode;
    double motor1RPM;
    double motor2RPM;

    /**
     * Creates a {@link HubbardShooterSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public HubbardShooterSubsystem(HubbardShooterHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);
        this.currentMode = "idle";

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("Motor1RPM", () -> motor1RPM, null);
            builder.addDoubleProperty("Motor2RPM", () -> motor2RPM, null);
            if (verboseLogging) {
                builder.addDoubleProperty("Motor1Amps", hardware::getMotor1Amps, null);
                builder.addDoubleProperty("Motor2Amps", hardware::getMotor2Amps, null);
                builder.addDoubleProperty("ShootingPower%", shootingPower::getAsDouble, null);
                builder.addDoubleProperty("IntakePower%", intakePower::getAsDouble, null);
                builder.addDoubleProperty("OuttakePower%", outtakePower::getAsDouble, null);
            }
        });
    }

    @Override
    public void periodic() {
        motor1RPM = hardware.getMotor1RPM();
        motor2RPM = hardware.getMotor2RPM();
    }

    /**
     * Converts power percentage (0-100) to voltage, clamped to safe range.
     */
    private double powerToVolts(double powerPercent) {
        return Util.clampVolts((powerPercent / 100.0) * Util.MAX_VOLTS);
    }

    private void cleanup() {
        hardware.stop();
        currentMode = "idle";
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that idles the shooter (motors stopped)
     */
    public Command idleCommand() {
        return run(() -> {
            currentMode = "idle";
            hardware.stop();
        });
    }

    /**
     * @return a command that runs intake mode (both motors pull fuel in)
     */
    public Command intakeCommand() {
        return startRun(
            () -> currentMode = "intake",
            () -> {
                double volts = powerToVolts(intakePower.getAsDouble());
                hardware.applyVoltage(volts, volts);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that runs outtake mode (both motors push fuel out)
     */
    public Command outtakeCommand() {
        return startRun(
            () -> currentMode = "outtake",
            () -> {
                double volts = powerToVolts(outtakePower.getAsDouble());
                hardware.applyVoltage(-volts, -volts);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that runs shooting mode (motors counter-rotate)
     */
    public Command shootCommand() {
        return startRun(
            () -> currentMode = "shooting",
            () -> {
                double volts = powerToVolts(shootingPower.getAsDouble());
                hardware.applyVoltage(volts, -volts);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that immediately stops both motors
     */
    public Command stopCommand() {
        return runOnce(() -> cleanup());
    }

    /**
     * Directly stops the shooter (for use outside command system).
     */
    public void stop() {
        cleanup();
    }

//endregion
}

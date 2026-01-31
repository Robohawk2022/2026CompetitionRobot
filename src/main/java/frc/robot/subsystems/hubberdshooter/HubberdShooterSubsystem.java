package frc.robot.subsystems.hubberdshooter;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.HubberdShooter.*;

/**
 * Subsystem for the HubberdShooter mechanism.
 * <p>
 * Controls two Falcon 500 motors with four operating modes:
 * <ul>
 *   <li><b>Off</b> - Motors stopped (0V)</li>
 *   <li><b>Intake</b> - Both motors same direction at configurable power</li>
 *   <li><b>Outtake</b> - Both motors opposite direction at configurable power</li>
 *   <li><b>Shooting</b> - Motors counter-rotate at configurable RPM</li>
 * </ul>
 */
public class HubberdShooterSubsystem extends SubsystemBase {

    /** Operating modes for the shooter */
    public enum Mode {
        OFF,
        INTAKE,
        OUTTAKE,
        SHOOTING
    }

    //region Implementation --------------------------------------------------------

    private final HubberdShooterHardware hardware;
    private String currentMode = "idle";
    private double motor1RPM = 0;
    private double motor2RPM = 0;

    /**
     * Creates a {@link HubberdShooterSubsystem}.
     *
     * @param hardware the hardware interface (required)
     */
    public HubberdShooterSubsystem(HubberdShooterHardware hardware) {
        this.hardware = Objects.requireNonNull(hardware);

        // Dashboard telemetry
        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("Motor1RPM", () -> motor1RPM, null);
            builder.addDoubleProperty("Motor2RPM", () -> motor2RPM, null);
            builder.addDoubleProperty("Motor1Amps", hardware::getMotor1Amps, null);
            builder.addDoubleProperty("Motor2Amps", hardware::getMotor2Amps, null);
            builder.addDoubleProperty("TargetRPM", shootingTargetRPM::getAsDouble, null);
            builder.addDoubleProperty("IntakePower%", intakePower::getAsDouble, null);
            builder.addDoubleProperty("OuttakePower%", outtakePower::getAsDouble, null);
        });
    }

    @Override
    public void periodic() {
        motor1RPM = hardware.getMotor1VelocityRPM();
        motor2RPM = hardware.getMotor2VelocityRPM();
    }

    /**
     * Converts power percentage (0-100) to voltage (-12 to +12).
     */
    private double powerToVolts(double powerPercent) {
        return Util.clampVolts((powerPercent / 100.0) * Util.MAX_VOLTS);
    }

    /**
     * Converts RPM to rotations per second.
     */
    private double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    /**
     * Cleanup helper - stops motors and resets state.
     */
    private void cleanup() {
        hardware.stop();
        currentMode = "idle";
    }

    //endregion

    //region Command factories -----------------------------------------------------

    /**
     * @return a command that stops both motors
     */
    public Command stopCommand() {
        return runOnce(() -> {
            currentMode = "stopped";
            hardware.stop();
        });
    }

    /**
     * @return a command that idles the shooter (motors stopped)
     */
    public Command idleCommand() {
        return startRun(
            () -> {
                currentMode = "idle";
                hardware.stop();
            },
            () -> {
                // do nothing - motors stay stopped
            }
        );
    }

    /**
     * @return a command that runs intake mode (both motors pull fuel in)
     */
    public Command intakeCommand() {
        return startRun(
            () -> {
                currentMode = "intake";
            },
            () -> {
                double volts = powerToVolts(intakePower.getAsDouble());
                // Both motors same direction (positive = pull in)
                hardware.applyVoltage(volts, volts);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that runs outtake mode (both motors push fuel out)
     */
    public Command outtakeCommand() {
        return startRun(
            () -> {
                currentMode = "outtake";
            },
            () -> {
                double volts = powerToVolts(outtakePower.getAsDouble());
                // Both motors same direction (negative = push out)
                hardware.applyVoltage(-volts, -volts);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    /**
     * @return a command that runs shooting mode (motors counter-rotate at target RPM)
     */
    public Command shootCommand() {
        return startRun(
            () -> {
                currentMode = "shooting";
                // Configure PID gains before starting velocity control
                hardware.configurePid(
                    kV.getAsDouble(),
                    kP.getAsDouble(),
                    kS.getAsDouble()
                );
            },
            () -> {
                double targetRPS = rpmToRps(shootingTargetRPM.getAsDouble());
                // Motors counter-rotate: motor1 positive, motor2 negative
                hardware.setVelocity(targetRPS, -targetRPS);
            }
        ).finallyDo(interrupted -> cleanup());
    }

    //endregion

    //region Direct control methods ------------------------------------------------
    // WARNING: These methods do not have automatic cleanup like commands.
    // If you call these, you MUST call stop() when done.

    /**
     * Directly stops the shooter (for use outside command system).
     */
    public void stop() {
        cleanup();
    }

    //endregion
}

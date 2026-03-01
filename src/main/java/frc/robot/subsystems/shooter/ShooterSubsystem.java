package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.Shooter.intakeRpm;
import static frc.robot.Config.Shooter.shootRpm;
import static frc.robot.Config.Shooter.shootSpeedTolerance;
import static frc.robot.Config.Shooter.stallSpeed;
import static frc.robot.Config.Shooter.stallTime;

/**
 * Shooter subsystem — controls the single shooter motor that launches balls.
 */
public class ShooterSubsystem extends SubsystemBase {

//region Implementation --------------------------------------------------------

    final ShooterHardware hardware;
    final MotorStatus shooterStatus;
    boolean shooterAtSpeed;

    public ShooterSubsystem(ShooterHardware hardware) {
        this.hardware = hardware;
        this.shooterStatus = new MotorStatus();
        this.shooterAtSpeed = false;

        SmartDashboard.putData("ShooterSubsystem", builder -> {
            shooterStatus.addToBuilder("ShooterMotor", builder);
            builder.addBooleanProperty("ShooterMotor/AtSpeed?", () -> shooterAtSpeed, null);
            builder.addDoubleProperty("ShooterMotor/Amps", hardware::getShooterAmps, null);
        });
    }

    @Override
    public void periodic() {
        shooterStatus.update(hardware.getShooterVelocity());
        shooterAtSpeed = Util.nearZero(
                shooterStatus.errorRpm,
                shootSpeedTolerance);
    }

    public boolean atSpeed() {
        return shooterAtSpeed;
    }

    /**
     * @return true if the shooter motor is currently stalled
     */
    public boolean motorStalled() {
        return shooterStatus.stalled;
    }

    /**
     * @return true if the shooter is at target speed
     */
    public boolean shooterAtSpeed() {
        return shooterAtSpeed;
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that will stop applying output to the shooter motor
     */
    public Command coast() {
        return velocityCommand(0.0);
    }

    /**
     * @return a command that will reset the PID when started, and then run
     * the shooter motor at the desired speed
     */
    public Command velocityCommand(double rpm) {
        return startRun(
                () -> {
                    shooterStatus.desiredRpm = rpm;
                    hardware.resetPid();
                },
                () -> hardware.applyRpm(rpm));
    }

    /**
     * @return a command that will spin up the shooter to the configured shoot RPM
     */
    public Command intakeCommand() {
        return defer(() -> velocityCommand(intakeRpm.getAsDouble()));
    }

    /**
     * @return a command that will spin up the shooter to the configured shoot RPM
     */
    public Command shootCommand() {
        return defer(() -> velocityCommand(shootRpm.getAsDouble()));
    }

//endregion

//region Helper ----------------------------------------------------------------

    /**
     * Represents the status of a single motor.
     */
    static class MotorStatus {

        final Debouncer debouncer;
        double currentRpm;
        double desiredRpm;
        double errorRpm;
        boolean stalled;

        public MotorStatus() {
            debouncer = new Debouncer(stallTime.getAsDouble(), DebounceType.kRising);
            currentRpm = 0.0;
            desiredRpm = 0.0;
            errorRpm = 0.0;
            stalled = false;
        }

        /**
         * Adds keys to the SmartDashboard to display motor status.
         */
        public void addToBuilder(String prefix, SendableBuilder builder) {
            builder.addDoubleProperty(prefix+"/CurrentRpm", () -> currentRpm, null);
            builder.addDoubleProperty(prefix+"/DesiredRpm", () -> desiredRpm, null);
            builder.addDoubleProperty(prefix+"/ErrorRpm", () -> errorRpm, null);
            builder.addBooleanProperty(prefix+"/Stalled?", () -> stalled, null);
        }

        /**
         * Updates motor status based on current/desired RPM.
         */
        public void update(double currentRpm) {
            this.currentRpm = currentRpm;
            if (desiredRpm != 0.0) {
                this.errorRpm = desiredRpm - currentRpm;
                this.stalled = debouncer.calculate(Math.abs(currentRpm) < stallSpeed.getAsDouble());
            } else {
                this.errorRpm = Double.NaN;
                this.stalled = false;
            }
        }
    }

//endregion

}

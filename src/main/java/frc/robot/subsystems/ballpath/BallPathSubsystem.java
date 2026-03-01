package frc.robot.subsystems.ballpath;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Config.BallPath.ejectSpeeds;
import static frc.robot.Config.BallPath.feedSpeeds;
import static frc.robot.Config.BallPath.intakeSpeeds;
import static frc.robot.Config.BallPath.stallSpeed;
import static frc.robot.Config.BallPath.stallTime;

/**
 * Ball-path subsystem — controls the intake, feeder, and agitator motors
 * that move balls through the robot into the shooter.
 */
public class BallPathSubsystem extends SubsystemBase {

//region Implementation --------------------------------------------------------

    final BallPathHardware hardware;
    final MotorStatus intakeStatus;
    final MotorStatus feederStatus;
    final MotorStatus agitatorStatus;

    public BallPathSubsystem(BallPathHardware hardware) {
        this.hardware = hardware;
        this.intakeStatus = new MotorStatus();
        this.feederStatus = new MotorStatus();
        this.agitatorStatus = new MotorStatus();

        SmartDashboard.putData("BallPathSubsystem", builder -> {
            intakeStatus.addToBuilder("IntakeMotor", builder);
            feederStatus.addToBuilder("FeederMotor", builder);
            agitatorStatus.addToBuilder("AgitatorMotor", builder);
        });
    }

    @Override
    public void periodic() {
        intakeStatus.update(hardware.getIntakeVelocity());
        feederStatus.update(hardware.getFeederVelocity());
        agitatorStatus.update(hardware.getAgitatorVelocity());
    }

    /**
     * @return true if any of our motors is currently stalled
     */
    public boolean motorStalled() {
        return intakeStatus.stalled
                || feederStatus.stalled
                || agitatorStatus.stalled;
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that will stop applying output to the motors
     */
    public Command coast() {
        return velocityCommand(0.0, 0.0, 0.0);
    }

    /**
     * @return a command that will reset the PID when started, and then run
     * the motors at the desired speeds
     */
    public Command velocityCommand(double intakeRpm,
                                   double feederRpm,
                                   double agitatorRpm) {
        return startRun(
                () -> {
                    intakeStatus.desiredRpm = intakeRpm;
                    feederStatus.desiredRpm = feederRpm;
                    agitatorStatus.desiredRpm = agitatorRpm;
                    hardware.resetPid();
                },
                () -> hardware.applyRpm(intakeRpm, feederRpm, agitatorRpm));
    }

    /**
     * @return a command that will run the motors at the intake speeds
     */
    public Command intakeCommand() {
        return defer(() -> velocityCommand(
                intakeSpeeds.intakeRpm.getAsDouble(),
                intakeSpeeds.feederRpm.getAsDouble(),
                intakeSpeeds.agitatorRpm.getAsDouble()));
    }

    /**
     * @return a command that will run the motors at the eject speeds
     * (intake, feeder, agitator spin backwards)
     */
    public Command ejectCommand() {
        return defer(() -> velocityCommand(
                -ejectSpeeds.intakeRpm.getAsDouble(),
                -ejectSpeeds.feederRpm.getAsDouble(),
                -ejectSpeeds.agitatorRpm.getAsDouble()));
    }

    /**
     * @return a command that will run the ball-path motors at feed speeds
     * to push balls into the spinning shooter
     */
    public Command feedCommand() {
        return defer(() -> velocityCommand(
                feedSpeeds.intakeRpm.getAsDouble(),
                feedSpeeds.feederRpm.getAsDouble(),
                feedSpeeds.agitatorRpm.getAsDouble()));
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
                this.errorRpm = 0.0;
                this.stalled = false;
            }
        }
    }

//endregion

}

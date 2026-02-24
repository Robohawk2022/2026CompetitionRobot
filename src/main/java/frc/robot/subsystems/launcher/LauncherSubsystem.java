package frc.robot.subsystems.launcher;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import static frc.robot.Config.Launcher.ejectSpeeds;
import static frc.robot.Config.Launcher.intakeSpeeds;
import static frc.robot.Config.Launcher.shootSpeedTolerance;
import static frc.robot.Config.Launcher.shootSpeeds;
import static frc.robot.Config.Launcher.stallSpeed;
import static frc.robot.Config.Launcher.stallTime;

/**
 * Launcher subsystem for the 2026 robot. Four spinning "wheel bars":
 * <ul>
 *     <li>Intake (in the gap in the front of the chassis)</li>
 *     <li>Feeder (directly behind intake and below shooter)</li>
 *     <li>Agitator (at the back in the hopper)</li>
 *     <li>Shooter (on the top)</li>
 * </ul>
 *
 * All motors turn in the same direction during intake and shooting; for
 * eject the intake, feeder and agitator will turn backwards.
 */
public class LauncherSubsystem extends SubsystemBase {

//region Implementation --------------------------------------------------------

    final LauncherHardware hardware;
    final MotorStatus intakeStatus;
    final MotorStatus feederStatus;
    final MotorStatus agitatorStatus;
    final MotorStatus shooterStatus;
    boolean shooterAtSpeed;

    public LauncherSubsystem(LauncherHardware hardware) {

        this.hardware = hardware;
        this.intakeStatus = new MotorStatus();
        this.feederStatus = new MotorStatus();
        this.agitatorStatus = new MotorStatus();
        this.shooterStatus = new MotorStatus();
        this.shooterAtSpeed = false;

        SmartDashboard.putData("LauncherSubsystem", builder -> {
            intakeStatus.addToBuilder("IntakeMotor", builder);
            feederStatus.addToBuilder("FeederMotor", builder);
            agitatorStatus.addToBuilder("AgitatorMotor", builder);
            shooterStatus.addToBuilder("ShooterMotor", builder);
            builder.addBooleanProperty("ShooterMotor/AtSpeed?", () -> shooterAtSpeed, null);
        });
    }

    @Override
    public void periodic() {

        // update status of all motors
        intakeStatus.update(hardware.getIntakeVelocity());
        feederStatus.update(hardware.getFeederVelocity());
        agitatorStatus.update(hardware.getAgitatorVelocity());
        shooterStatus.update(hardware.getShooterVelocity());

        // is the shooter at speed?
        shooterAtSpeed = Util.nearZero(
                shooterStatus.errorRpm,
                shootSpeedTolerance);
    }

    /**
     * @return true if any of our motors is currently stalled
     */
    public boolean motorStalled() {
        return intakeStatus.stalled
                || feederStatus.stalled
                || agitatorStatus.stalled
                || shooterStatus.stalled;
    }

    /**
     * @return true if the shooter is at target speed
     */
    public boolean shooterAtSpeed() {
        return shooterAtSpeed;
    }

//endregion

//region Triggers & Commands ---------------------------------------------------

    /**
     * @return a command that will stop applying output to the motors
     */
    public Command coast() {
        return velocityCommand(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * @return a command that will reset the PID when started, and then run
     * the motors at the desired speeds
     */
    public Command velocityCommand(double intakeRpm,
                                   double feederRpm,
                                   double agitatorRpm,
                                   double shooterRpm) {
        return startRun(
                () -> {
                    intakeStatus.desiredRpm = intakeRpm;
                    feederStatus.desiredRpm = feederRpm;
                    agitatorStatus.desiredRpm = agitatorRpm;
                    shooterStatus.desiredRpm = shooterRpm;
                    hardware.resetPid();
                },
                () -> hardware.applyRpm(intakeRpm, feederRpm, agitatorRpm, shooterRpm));
    }

    /**
     * @return a command that will run the motors at the desired speed
     * for intake
     */
    public Command intakeCommand() {
        return defer(() -> velocityCommand(
                intakeSpeeds.intakeRpm.getAsDouble(),
                intakeSpeeds.feederRpm.getAsDouble(),
                intakeSpeeds.agitatorRpm.getAsDouble(),
                intakeSpeeds.shooterRpm.getAsDouble()));
    }

    /**
     * @return a command that will run the motors at the desired speed
     * for ejecting
     */
    public Command ejectCommand() {
        return defer(() -> velocityCommand(

                // intake, feeder and agitator spin backwards during eject
                -ejectSpeeds.intakeRpm.getAsDouble(),
                -ejectSpeeds.feederRpm.getAsDouble(),
                -ejectSpeeds.agitatorRpm.getAsDouble(),

                ejectSpeeds.shooterRpm.getAsDouble()));
    }

    /**
     * @return a command that will run only the shooter motor at the desired
     * speed for shooting
     */
    public Command spinUpCommand() {
        return defer(() -> velocityCommand(
                0.0,
                0.0,
                0.0,
                shootSpeeds.shooterRpm.getAsDouble()));
    }

    /**
     * @return a command that will run the motors at the desired speed
     * for shooting
     */
    public Command shootCommand() {
        return defer(() -> velocityCommand(
                shootSpeeds.intakeRpm.getAsDouble(),
                shootSpeeds.feederRpm.getAsDouble(),
                shootSpeeds.agitatorRpm.getAsDouble(),
                shootSpeeds.shooterRpm.getAsDouble()));
    }

//endregion

//region Helper ----------------------------------------------------------------

    /**
     * Represents the status of a single motor
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
         * Adds keys to the SmartDashboard to display motor status
         */
        public void addToBuilder(String prefix, SendableBuilder builder) {
            builder.addDoubleProperty(prefix+"/CurrentRpm", () -> currentRpm, null);
            builder.addDoubleProperty(prefix+"/DesiredRpm", () -> desiredRpm, null);
            builder.addDoubleProperty(prefix+"/ErrorRpm", () -> errorRpm, null);
            builder.addBooleanProperty(prefix+"/Stalled?", () -> stalled, null);
        }

        /**
         * Updates motor status based on current/desired RPM
         */
        public void update(double currentRpm) {

            this.currentRpm = currentRpm;

            // if we have a target speed, we calculate error (target - current)
            // and check to see if we've been stalling for too long
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

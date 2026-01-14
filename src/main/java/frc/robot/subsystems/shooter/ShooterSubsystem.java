package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import static frc.robot.Config.Shooter.d;
import static frc.robot.Config.Shooter.p;
import static frc.robot.Config.Shooter.v;

/**
 * Implements a shooter. The "heavy lifting" of the feedback and feedforward
 * is handled by the motor controllers, which are managed by the
 * {@link ShooterHardware} implementation.
 */
public class ShooterSubsystem implements Subsystem {

//region Implementation --------------------------------------------------------

    final ShooterHardware hardware;
    double currentRps;
    double currentFps;
    double targetRps;
    double targetFps;
    double targetVolts;
    boolean brakeMode;

    public ShooterSubsystem(ShooterHardware hardware) {

        this.hardware = hardware;
        this.brakeMode = false;

        SmartDashboard.putData("ShooterSubsystem", builder -> {
            builder.addDoubleProperty("FeetPerSecond", () -> currentFps, null);
            builder.addDoubleProperty("RevolutionsPerSecond", () -> currentRps, null);
            builder.addBooleanProperty("Brake?", () -> brakeMode, this::setBrakeMode);
            // TODO what else would be useful to display?
        });
    }

    /*
     * We don't want to call the motor every cycle to fetch or update the
     * brake mode. So we will "remember"" it in a variable, and only send
     * commands to the hardware when it changes.
     */
    private void setBrakeMode(boolean newMode) {
        if (newMode != brakeMode) {
            brakeMode = newMode;
            hardware.setBrakeMode(brakeMode);
        }
    }

    @Override
    public void periodic() {

        // get the current speed of the motor in revolutions per second
        currentRps = hardware.getMotorRevolutionsPerSecond();

        // convert the speed in revolutions per second to a speed in
        // feet per second using the gear ration and wheel diameter
        currentFps = -1.0; // TODO implement me
    }

    public void openLoop(double volts) {

        // clear the target speeds
        targetRps = Double.NaN;
        targetFps = Double.NaN;

        // clamp and apply the target voltage
        targetVolts = Util.clampVolts(volts);
        hardware.applyVolts(targetVolts);
    }

    public void closedLoop(double feetPerSecond) {

        // clear the target volts
        targetVolts = Double.NaN;

        // calculate the target revolutions per second from the target feet
        // per second. it's a little expensive to do this, we'll only do it
        // when the target changes.
        if (targetFps != feetPerSecond) {

            targetFps = feetPerSecond;
            targetRps = 0.0; // TODO implement me

            // if we notice that our setpoint has changed, and we're not in a
            // competition, we're probably in the lab tuning the robot. this is
            // a good time to reset our PID parameters from configuration.
            if (!Util.isCompetition()) {
                hardware.resetPid(
                        p.getAsDouble(),
                        d.getAsDouble(),
                        v.getAsDouble());
            }
        }

        // apply the target speed
        hardware.applySpeed(targetRps);
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that will "idle" the shooter wheel by applying 0
     * volts to the motor
     */
    public Command idleCommand() {
        return voltsCommand(0.0);
    }

    /**
     * @param volts a voltage level (range is [-12, 12])
     * @return a command that will continuously apply the specified number of
     * volts to the motors
     */
    public Command voltsCommand(double volts) {
        throw new UnsupportedOperationException("TODO implement me");
    }

    /**
     * @param input joystick input supplier (range is [-1.0, 1.0])
     * @return a command that will read joystick input and run the motors at
     * the corresponding voltage
     */
    public Command teleopCommand(DoubleSupplier input) {
        throw new UnsupportedOperationException("TODO implement me");
    }

    /**
     * @param preset a preset value
     * @return a command that will run the shooter at the speed corresponding
     * to that preset
     */
    public Command presetCommand(ShooterPreset preset) {

        Objects.requireNonNull(preset);

        // using a deferred command here means that we will pick up the
        // current configuration value whenever the command is run, in
        // case it changes in between runs
        return defer(() -> speedCommand(preset.feetPerSecond()));
    }

    /**
     * @param feetPerSecond desired speed in feet per second
     * @return a command that will run the shooter in closed-loop mode and
     * hold the wheel at the specified speed
     */
    public Command speedCommand(double feetPerSecond) {
        throw new UnsupportedOperationException("TODO implement me");
    }

//endregion

}

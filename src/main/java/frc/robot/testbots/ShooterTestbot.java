package frc.robot.testbots;

import java.util.Set;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameController;
import frc.robot.subsystems.shooter.ShooterHardware;
import frc.robot.subsystems.shooter.ShooterHardwareSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Testbot for the {@link ShooterSubsystem}
 */
public class ShooterTestbot extends TimedRobot {

    final ShooterHardware hardware;
    final ShooterSubsystem shooter;
    final GameController controller;
    double desiredSpeed;

    public ShooterTestbot() {

        // we can run with either the simulation hardware or the real thing
        hardware = new ShooterHardwareSim();
        // hardware = new ShooterHardwareRev();

        shooter = new ShooterSubsystem(hardware);
        controller = new GameController(0);

        // default behavior for the shooter is to be idle (0 volts)
        shooter.setDefaultCommand(shooter.idleCommand());

        // this says that, if someone is pressing the left joystick up or
        // down, we should enter teleop mode on the shooter (you can use this
        // to get a feel for how fast the motors can run)
        new Trigger(() -> Math.abs(controller.getLeftY()) > 0.1)
                .whileTrue(shooter.teleopCommand(controller::getLeftY));

        // this will run the shooter at the desired speed whenever someone
        // holds down the A button (you can use this for tuning)
        // using defer() so the command reads the current desiredSpeed each time
        // the button is pressed, not the value at binding time
        controller.a().whileTrue(
            Commands.defer(() -> shooter.speedCommand(desiredSpeed), Set.of(shooter)));

        // this displays desired speed in the dashboard so you can configure
        // it during testing
        SmartDashboard.putData("ShooterTestbot", builder -> {
            builder.addDoubleProperty("DesiredFeetPerSecond", () -> desiredSpeed, val -> desiredSpeed = val);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

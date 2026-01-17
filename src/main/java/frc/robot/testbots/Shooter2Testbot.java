package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter2.ShooterSubsystem2;

public class Shooter2Testbot extends TimedRobot {

    public static final int CAN1 = 1;
    public static final int CAN2 = 2;

    final ShooterSubsystem2 shooter;
    final CommandXboxController controller;
    double speed1;
    double speed2;

    public Shooter2Testbot() {

        shooter = new ShooterSubsystem2(CAN1, CAN2);
        controller = new CommandXboxController(0);
        speed1 = 4000.0;
        speed2 = 4000.0;

        SmartDashboard.putData("ShooterTestbot", builder -> {
            builder.addDoubleProperty("Speed1", () -> speed1, val -> speed1 = val);
            builder.addDoubleProperty("Speed2", () -> speed2, val -> speed2 = val);
        });

        shooter.setDefaultCommand(shooter.idleCommand());

        Command teleop = shooter.teleopCommand(controller::getLeftY);
        controller.rightBumper().whileTrue(teleop);
        controller.leftBumper().whileTrue(shooter.defer(() ->
            shooter.runAt(speed1, speed2)));
    }

    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;

public class AutonomousTestbot extends TimedRobot {

    final SwerveSubsystem swerve;
    final AutonomousSubsystem auto;

    public AutonomousTestbot() {

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        auto = new AutonomousSubsystem(swerve);

        CommandXboxController controller = new CommandXboxController(0);
        controller.a().onTrue(auto.generateCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

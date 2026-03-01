package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.ballpath.BallPathHardwareSim;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterHardwareSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;

public class AutonomousTestbot extends TimedRobot {

    final SwerveSubsystem swerve;
    final ShooterSubsystem shooter;
    final BallPathSubsystem ballPath;
    final LEDSubsystem led;
    final AutonomousSubsystem auto;

    public AutonomousTestbot() {

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        shooter = new ShooterSubsystem(new ShooterHardwareSim());
        ballPath = new BallPathSubsystem(new BallPathHardwareSim());
        led = new LEDSubsystem(new LEDHardwareSim());
        auto = new AutonomousSubsystem(swerve, shooter, ballPath, led);

        CommandXboxController controller = new CommandXboxController(0);
        controller.a().onTrue(auto.generateCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

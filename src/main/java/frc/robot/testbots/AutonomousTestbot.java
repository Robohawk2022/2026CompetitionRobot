package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GameController;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.ballpath.BallPathHardwareSim;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSim;
import frc.robot.subsystems.limelight.LimelightSubsystem;
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
    final LimelightSubsystem limelight;
    final LimelightSim limelightSim;

    public AutonomousTestbot() {

        GameController controller = new GameController(0);

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        swerve.setDefaultCommand(swerve.teleopCommand(controller));

        limelight = new LimelightSubsystem(swerve);
        limelightSim = new LimelightSim();

        shooter = new ShooterSubsystem(new ShooterHardwareSim());
        shooter.setDefaultCommand(shooter.idleCommand());

        ballPath = new BallPathSubsystem(new BallPathHardwareSim());
        ballPath.setDefaultCommand(ballPath.coast());

        led = new LEDSubsystem(new LEDHardwareSim());
        led.setDefaultCommand(led.show(() -> RobotContainer.idleLedSignalCalculator(
            swerve, 
            limelight)));

        auto = new AutonomousSubsystem(swerve,
                shooter,
                ballPath,
                led);

        controller.a().whileTrue(ShootingCommands.intakeMode(led, ballPath, shooter));
        controller.b().whileTrue(ShootingCommands.shootMode(led, ballPath, shooter));
        controller.x().whileTrue(ShootingCommands.orientToShoot(led, swerve));
        controller.y().whileTrue(auto.generateCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
         limelightSim.update(swerve.getPose());
    }
}

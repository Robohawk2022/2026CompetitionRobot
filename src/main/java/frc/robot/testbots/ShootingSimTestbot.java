package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.subsystems.ballpath.BallPathHardwareSim;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.limelight.LimelightSim;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterHardwareSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;

public class ShootingSimTestbot extends TimedRobot {

    final SwerveSubsystem swerve;
    final ShooterSubsystem shooter;
    final BallPathSubsystem ballPath;
    final LimelightSubsystem limelight;
    final LimelightSim limelightSim;

    public ShootingSimTestbot() {

        GameController controller = new GameController(0);

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        swerve.setDefaultCommand(new SwerveTeleopCommand(swerve, controller));

        shooter = new ShooterSubsystem(new ShooterHardwareSim());
        shooter.setDefaultCommand(shooter.coast());

        ballPath = new BallPathSubsystem(new BallPathHardwareSim());
        ballPath.setDefaultCommand(ballPath.coast());

        limelight = new LimelightSubsystem(swerve);
        limelightSim = new LimelightSim();

        controller.a().onTrue(ShootingCommands.orientToShoot(swerve));
        controller.b().whileTrue(ShootingCommands.jiggle(swerve));
        controller.x().whileTrue(ShootingCommands.driveAndShootCommand(swerve, shooter, ballPath));
        controller.y().onTrue(swerve.resetPoseCommand(oldPose -> Pose2d.kZero));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
         limelightSim.update(swerve.getPose());
    }
}

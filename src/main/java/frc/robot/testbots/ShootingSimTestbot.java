package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.limelight.LimelightSim;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;

public class ShootingSimTestbot extends TimedRobot {

    final SwerveSubsystem swerve;
    final LauncherSubsystem launcher;
    final LimelightSubsystem limelight;
    final LimelightSim limelightSim;

    public ShootingSimTestbot() {

        GameController controller = new GameController(0);

        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        swerve.setDefaultCommand(new SwerveTeleopCommand(swerve, controller));

        launcher = new LauncherSubsystem(new LauncherHardwareSim());
        launcher.setDefaultCommand(launcher.coast());

        limelight = new LimelightSubsystem(swerve);
        limelightSim = new LimelightSim();

        controller.a().onTrue(ShootingCommands.orientToShoot(swerve));
        controller.b().whileTrue(ShootingCommands.jiggleCommand(swerve));
        controller.x().onTrue(swerve.resetPoseCommand(oldPose -> Pose2d.kZero));
        // controller.y().onTrue(ShootingCommands.driveAndShootCommand(swerve, launcher));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // limelightSim.update(swerve.getPose());
    }
}

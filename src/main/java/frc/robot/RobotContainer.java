// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.agitator.AgitatorHardwareRev;
import frc.robot.subsystems.agitator.AgitatorHardwareSim;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.CommandLogger;
import frc.robot.util.Field;
import frc.robot.util.Util;

public class RobotContainer {

  final GameController driver = new GameController(0);
  final GameController operator = new GameController(1);

  final CommandSwerveDrivetrain drivetrain;
  final SwerveSubsystem swerve;
  final LimelightSubsystem limelight;
  final AutonomousSubsystem auto;
  final LauncherSubsystem launcher;
  final AgitatorSubsystem agitator;

  public RobotContainer() {

    CommandLogger.enable();
    CommandLogger.addController("Driver", driver);
    CommandLogger.addController("Operator", operator);

    // create CTRE drivetrain and wrap it in SwerveSubsystem
    drivetrain = TunerConstants.createDrivetrain();
    swerve = new SwerveSubsystem(drivetrain);
    limelight = new LimelightSubsystem(swerve);
    auto = new AutonomousSubsystem(swerve);

    // create launcher and agitator
    boolean sim = Robot.isSimulation();
    launcher = new LauncherSubsystem(sim ? new LauncherHardwareSim() : new LauncherHardwareRev());
    agitator = new AgitatorSubsystem(sim ? new AgitatorHardwareSim() : new AgitatorHardwareRev());

    launcher.setDefaultCommand(launcher.idleCommand());
    agitator.setDefaultCommand(agitator.idleCommand());

    // shooting distance telemetry
    SmartDashboard.putData("Shooting", builder -> {
        builder.addDoubleProperty("DistanceToHubFeet", () ->
            Util.feetBetween(swerve.getPose(), Field.getHubCenter()), null);
        builder.addStringProperty("SelectedShotType", () -> {
            double dist = Util.feetBetween(swerve.getPose(), Field.getHubCenter());
            return ShootingCommands.selectShotType(dist).name();
        }, null);
    });

    // configure driver controls
    configureBindings();
  }

  private void configureBindings() {

    // default command: normal teleop drive
    swerve.setDefaultCommand(swerve.driveCommand(driver));

    // hold left bumper for orbit mode (face and orbit around target)
    driver.leftBumper()
        .whileTrue(swerve.orbitCommand(driver));

    // zero pose on left click, accept vision pose on right click
    driver.leftStick().onTrue(swerve.zeroPoseCommand());
    driver.rightStick().onTrue(limelight.resetPoseFromVisionCommand());

    //==========================================================================
    // Operator controls - shooting
    //==========================================================================

    // right trigger: distance-based auto-shot (hold to shoot)
    operator.rightTrigger()
        .whileTrue(ShootingCommands.makeShotCommand(swerve, launcher, agitator));

    // A: manual close-range shot (hold)
    operator.a()
        .whileTrue(ShootingCommands.vomitCommand(launcher, agitator,
            ShootingCommands.ShotType.CLOSE_RANGE));

    // B: manual long-range shot (hold)
    operator.b()
        .whileTrue(ShootingCommands.vomitCommand(launcher, agitator,
            ShootingCommands.ShotType.LONG_RANGE));

    // X: intake (hold) - lower wheel + agitator forward
    operator.x()
        .whileTrue(Commands.parallel(
            launcher.intakeWheelCommand(),
            agitator.forwardCommand()));

    // Y: emergency stop
    operator.y()
        .onTrue(Commands.parallel(
            launcher.stopCommand(),
            agitator.stopCommand()));
  }

  public Command getAutonomousCommand() {
    return auto.generateCommand();
  }
}

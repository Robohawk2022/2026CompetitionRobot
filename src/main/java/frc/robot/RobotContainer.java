// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.CommandLogger;

public class RobotContainer {

  final GameController driver = new GameController(0);
  final GameController operator = new GameController(1);

  final CommandSwerveDrivetrain drivetrain;
  final SwerveSubsystem swerve;
  final LimelightSubsystem limelight;
  final AutonomousSubsystem auto;
  final LauncherSubsystem launcher;

  public RobotContainer() {

    CommandLogger.enable();
    CommandLogger.addController("Driver", driver);
    CommandLogger.addController("Operator", operator);

    // create CTRE drivetrain and wrap it in SwerveSubsystem
    drivetrain = TunerConstants.createDrivetrain();
    swerve = new SwerveSubsystem(drivetrain);
    limelight = new LimelightSubsystem(swerve);
    auto = new AutonomousSubsystem(swerve);

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
  }

  public Command getAutonomousCommand() {
    return auto.generateCommand();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveHardware;
import frc.robot.subsystems.swerve.SwerveHardwareCTRE;
import frc.robot.subsystems.swerve.SwerveHardwareSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveTeleopSpeedSupplier;
import frc.robot.util.CommandLogger;

public class RobotContainer {

  final CommandXboxController driver = new CommandXboxController(0);
  final CommandXboxController operator = new CommandXboxController(1);

  final SwerveSubsystem swerve;

  public RobotContainer() {
    CommandLogger.enable();
    CommandLogger.addController("Driver", driver);
    CommandLogger.addController("Operator", operator);

    // create hardware and subsystem
    SwerveHardware swerveHardware = Robot.isSimulation()
        ? new SwerveHardwareSim()
        : new SwerveHardwareCTRE();
    swerve = new SwerveSubsystem(swerveHardware);

    // set default command with turbo/sniper triggers
    SwerveTeleopSpeedSupplier driverInput = new SwerveTeleopSpeedSupplier(driver);
    swerve.setDefaultCommand(driverInput.driveCommand(swerve));

    configureBindings();
  }

  private void configureBindings() {
    // reset heading when both sticks clicked
    driver.leftStick()
        .and(driver.rightStick())
        .onTrue(swerve.resetPoseCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

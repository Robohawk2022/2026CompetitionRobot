// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.CommandLogger;

public class RobotContainer {

  final CommandXboxController driver = new CommandXboxController(0);
  final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    CommandLogger.enable();
    CommandLogger.addController("Driver", driver);
    CommandLogger.addController("Operator", operator);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

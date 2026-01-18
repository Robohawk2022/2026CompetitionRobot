// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.testbots.SwerveTestbot;

import java.util.function.Supplier;

public final class Main {
  private Main() {}

  public static void main(String... args) {
      RobotBase.startRobot(SwerveTestbot::new);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.testbots.SysIdTestbot;
import frc.robot.testbots.ShooterTestbot;
import frc.robot.testbots.SwerveTestbot;
//ssssssss
//eeeee3e

import java.util.function.Supplier;

public final class Main {
  private Main() {}

  public static void main(String... args) {
      String robotClassName = System.getenv("ROBOT_CLASS");
      if (robotClassName != null && !robotClassName.isEmpty()) {
          try {
              Class<?> robotClass = Class.forName(robotClassName);
              Supplier<TimedRobot> supplier = () -> {
                  try {
                      return (TimedRobot) robotClass.getDeclaredConstructor().newInstance();
                  } catch (Exception e) {
                      throw new RuntimeException("Failed to instantiate " + robotClassName, e);
                  }
              };
              RobotBase.startRobot(supplier);
          } catch (ClassNotFoundException e) {
              System.err.println("Robot class not found: " + robotClassName);
              System.err.println("Falling back to Robot");
              RobotBase.startRobot(SysIdTestbot::new);
          }
      } else {
          RobotBase.startRobot(SwerveTestbot::new);
      }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.function.Supplier;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    // Check for testbot selection via ROBOT_CLASS environment variable
    // Set by gradle with: ./gradlew simulateJava -Probot=XxxTestbot
    String robotClass = System.getenv("ROBOT_CLASS");

    if (robotClass != null && !robotClass.isEmpty()) {
      System.out.println(">>> Starting robot class: " + robotClass);
      RobotBase.startRobot(createRobotSupplier(robotClass));
    } else {
      RobotBase.startRobot(Robot::new);
    }
  }

  @SuppressWarnings("unchecked")
  private static Supplier<TimedRobot> createRobotSupplier(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      return () -> {
        try {
          return (TimedRobot) clazz.getDeclaredConstructor().newInstance();
        } catch (Exception e) {
          throw new RuntimeException("Failed to instantiate robot: " + className, e);
        }
      };
    } catch (ClassNotFoundException e) {
      throw new RuntimeException("Robot class not found: " + className, e);
    }
  }
}

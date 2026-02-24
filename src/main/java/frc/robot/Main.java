// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.testbots.AutonomousTestbot;
import frc.robot.testbots.LauncherTestbot;
import frc.robot.testbots.ShootingSimTestbot;
import frc.robot.testbots.SwerveTestbot;

public final class Main {

    private Main() {}

    public static void main(String... args) {
        // RobotBase.startRobot(LEDTestbot::new);
        // RobotBase.startRobot(SysIdTestbot::new);
        RobotBase.startRobot(SwerveTestbot::new);
        // RobotBase.startRobot(VisionSimTestbot::new);
        // RobotBase.startRobot(Robot::new);
        // RobotBase.startRobot(LauncherTestbot::new);
    //    RobotBase.startRobot(ShootingSimTestbot::new);
        // RobotBase.startRobot(AutonomousTestbot::new);
    }
}


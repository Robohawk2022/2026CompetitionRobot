// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.testbots.HubbardShooterTestbot;
import frc.robot.testbots.LEDTestbot;
import frc.robot.testbots.IntakeFrontTestbot;
import frc.robot.testbots.SysIdTestbot;
import frc.robot.testbots.VisionSimTestbot;
import frc.robot.testbots.SwerveTestbot;


import java.util.function.Supplier;

public final class Main {

    private Main() {}

    public static void main(String... args) {
        // RobotBase.startRobot(LEDTestbot::new);
        // RobotBase.startRobot(IntakeFrontTestbot::new);
        // RobotBase.startRobot(SysIdTestbot::new);
        // RobotBase.startRobot(SwerveTestbot::new);
        // RobotBase.startRobot(VisionSimTestbot::new);
        // RobotBase.startRobot(Robot::new);
        RobotBase.startRobot(HubbardShooterTestbot::new);
    }
}


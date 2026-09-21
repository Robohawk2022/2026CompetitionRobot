// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CommandLogger;
import frc.robot.util.PowerLog;

public class Robot extends TimedRobot {

    private final RobotContainer container;
    private Command autoCommand;

    public Robot() {

        // Logging to disk. Before this, nothing was written to the roboRIO:
        // every dashboard value evaporated at the end of the match.
        //
        // DataLogManager records every NetworkTables topic (so all the
        // existing SmartDashboard telemetry) plus console output to a
        // .wpilog in /home/lvuser/logs, or on a USB stick if one is plugged
        // into the roboRIO. DriverStation.startDataLog adds enable/mode and
        // joystick data. SignalLogger starts the CTRE .hoot log at boot
        // rather than waiting for first enable.
        //
        // Retrieve with the WPILib Data Log Tool, AdvantageScope, or
        //   scp lvuser@10.33.73.2:/home/lvuser/logs/*.wpilog .
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        SignalLogger.start();
        PowerLog.start();

        container = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        CommandLogger.pollButtons();
        PowerLog.update();
    }

//region Auto ------------------------------------------------------------------

    @Override
    public void autonomousInit() {
        autoCommand = container.getAutonomousCommand();
        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

//endregion

//region Teleop ----------------------------------------------------------------

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

//endregion

//region Other -----------------------------------------------------------------

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }
}

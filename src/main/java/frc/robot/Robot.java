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

public class Robot extends TimedRobot {

    private final RobotContainer container;
    private Command autoCommand;

    public Robot() {

        // persist NetworkTables telemetry and DS state to disk (a USB stick if one is
        // plugged in, otherwise /home/lvuser/logs). without this, everything the
        // subsystems publish is live-only and there is nothing to read back after a
        // brownout
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        // phoenix signal logging records TalonFX supply and stator current straight off
        // the CAN bus at full rate, into .hoot files readable in Tuner X. this is what
        // lets us tell battery sag apart from drivetrain draw after the fact
        SignalLogger.start();

        container = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        CommandLogger.pollButtons();
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

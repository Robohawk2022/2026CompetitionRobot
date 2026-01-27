// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.SwerveHardwareCTRE;
import frc.robot.subsystems.swerve.SwerveHardwareSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.CommandLogger;
import frc.robot.util.Util;

import static frc.robot.Config.PathPlanner.*;
import static frc.robot.Config.Swerve.*;

public class RobotContainer {

  final GameController driver = new GameController(0);
  final GameController operator = new GameController(1);

  final SwerveSubsystem swerve;
  final LimelightSubsystem limelight;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    CommandLogger.enable();
    CommandLogger.addController("Driver", driver);
    CommandLogger.addController("Operator", operator);

    swerve = new SwerveSubsystem(Robot.isSimulation()
            ? new SwerveHardwareSim()
            : new SwerveHardwareCTRE());

    limelight = new LimelightSubsystem(swerve);

    // configure PathPlanner AutoBuilder
    configurePathPlanner();

    // build auto chooser (must be after AutoBuilder is configured)
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // configure driver controls
    configureBindings();
  }

  /**
   * Configures PathPlanner's AutoBuilder for swerve drive path following.
   */
  private void configurePathPlanner() {
    // Create module config for a single swerve module
    ModuleConfig moduleConfig = new ModuleConfig(
        WHEEL_DIAMETER_METERS / 2.0,  // wheel radius
        MAX_WHEEL_SPEED_MPS,          // max module speed
        WHEEL_COF,                    // wheel coefficient of friction
        DCMotor.getKrakenX60(1),      // drive motor
        DRIVE_CURRENT_LIMIT_AMPS,     // current limit
        1                             // number of motors per module
    );

    // Create robot config with module positions (same order as kinematics: FL, FR, BL, BR)
    RobotConfig robotConfig = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        moduleConfig,
        new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),   // FL
        new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),  // FR
        new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),  // BL
        new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)  // BR
    );

    // Configure AutoBuilder
    AutoBuilder.configure(
        swerve::getPose,                      // pose supplier
        swerve::resetPose,                    // pose reset consumer
        swerve::getCurrentSpeeds,             // chassis speeds supplier
        speed -> swerve.driveRobotRelative("auto", speed),   // robot-relative chassis speeds consumer
        new PPHolonomicDriveController(       // path following controller
            new PIDConstants(                 // translation PID
                translationKP.getAsDouble(),
                translationKI.getAsDouble(),
                translationKD.getAsDouble()
            ),
            new PIDConstants(                 // rotation PID
                rotationKP.getAsDouble(),
                rotationKI.getAsDouble(),
                rotationKD.getAsDouble()
            )
        ),
        robotConfig,                          // robot configuration
        this::shouldFlipPath,                 // path flip supplier for red alliance
        swerve                                // drive subsystem requirement
    );

    if (enableLogging.getAsBoolean()) {
      Util.log("PathPlanner AutoBuilder configured for swerve drive");
    }
  }

  /**
   * @return true if paths should be flipped for red alliance
   */
  private boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
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
    return autoChooser.getSelected();
  }
}

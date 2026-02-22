// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakefront.IntakeFrontHardwareSim;
import frc.robot.subsystems.intakefront.IntakeFrontHardwareSparkMax;
import frc.robot.subsystems.intakefront.IntakeFrontSubsystem;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.led.LEDHardwareBlinkin;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.CommandLogger;
import frc.robot.util.Field;
import frc.robot.util.Util;

public class RobotContainer {

  final GameController driver = new GameController(0);
  final GameController operator = new GameController(1);

  // core subsystems (same as SwerveTestbot)
  final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  final SwerveSubsystem swerve = new SwerveSubsystem(drivetrain);
  final LimelightSubsystem limelight = new LimelightSubsystem(swerve);

  // launcher + intake + LED
  final LauncherSubsystem launcher = new LauncherSubsystem(Robot.isSimulation()
          ? new LauncherHardwareSim() : new LauncherHardwareRev());
  final IntakeFrontSubsystem intake = new IntakeFrontSubsystem(Robot.isSimulation()
          ? new IntakeFrontHardwareSim() : new IntakeFrontHardwareSparkMax());
  final LEDSubsystem led = new LEDSubsystem(Robot.isSimulation()
          ? new LEDHardwareSim() : new LEDHardwareBlinkin());

  public RobotContainer() {

    CommandLogger.enable();
    CommandLogger.addController("Driver", driver);
    CommandLogger.addController("Operator", operator);

    // default commands
    swerve.setDefaultCommand(swerve.driveCommand(driver));
    launcher.setDefaultCommand(launcher.idleCommand());

    // LED distance to hub
    led.setDistanceSupplier(() -> Util.feetBetween(swerve.getPose(), Field.getHubCenter()));
    intake.setStallCallback(stalled -> {
        if (stalled) led.flash(LEDSignal.INTAKE_FULL, 2.0);
    });

    // driver bindings
    driver.leftBumper().whileTrue(swerve.orbitCommand(driver));
    driver.rightBumper().whileTrue(swerve.aimAtHubCommand(driver));
    driver.y().onTrue(ShootingCommands.driveAndShootCommand(swerve, launcher));
    driver.leftTrigger().whileTrue(swerve.jiggleCommand());
    driver.leftStick().onTrue(swerve.zeroPoseCommand());
    driver.rightStick().onTrue(limelight.resetPoseFromVisionCommand());

    // operator bindings
    operator.a().whileTrue(Commands.parallel(
        launcher.intakeCommand(),
        intake.intakeCommand()));
    operator.b().whileTrue(launcher.shootCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

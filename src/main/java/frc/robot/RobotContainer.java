// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.auto.AutonomousSubsystem;
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

  final CommandSwerveDrivetrain drivetrain;
  final SwerveSubsystem swerve;
  final LimelightSubsystem limelight;
  final AutonomousSubsystem auto;
  final LEDSubsystem led;
  final LauncherSubsystem launcher;
  final IntakeFrontSubsystem intake;

  public RobotContainer() {

    CommandLogger.enable();
    CommandLogger.addController("Driver", driver);
    CommandLogger.addController("Operator", operator);

    System.out.println(">>> Creating drivetrain...");
    drivetrain = TunerConstants.createDrivetrain();
    System.out.println(">>> Creating swerve...");
    swerve = new SwerveSubsystem(drivetrain);
    System.out.println(">>> Creating limelight...");
    limelight = new LimelightSubsystem(swerve);
    System.out.println(">>> Creating auto...");
    auto = new AutonomousSubsystem(swerve);
    System.out.println(">>> Creating LED...");
    led = new LEDSubsystem(Robot.isSimulation()
            ? new LEDHardwareSim()
            : new LEDHardwareBlinkin());
    System.out.println(">>> Creating launcher...");
    launcher = new LauncherSubsystem(Robot.isSimulation()
            ? new LauncherHardwareSim()
            : new LauncherHardwareRev());
    System.out.println(">>> Creating intake...");
    intake = new IntakeFrontSubsystem(Robot.isSimulation()
            ? new IntakeFrontHardwareSim()
            : new IntakeFrontHardwareSparkMax());

    // LED always shows distance to hub using odometry
    led.setDistanceSupplier(() -> Util.feetBetween(swerve.getPose(), Field.getHubCenter()));

    // flash orange when intake stalls (hopper full)
    intake.setStallCallback(stalled -> {
        if (stalled) led.flash(LEDSignal.INTAKE_FULL, 2.0);
    });

    // default commands
    launcher.setDefaultCommand(launcher.idleCommand());

    // configure driver controls
    configureBindings();

    System.out.println(">>> RobotContainer initialized successfully");
    System.out.println(">>> Y=4 RightBumper=6 LeftBumper=5 (Xbox sim buttons)");
  }

  private void configureBindings() {

    // default command: normal teleop drive
    swerve.setDefaultCommand(swerve.driveCommand(driver));

    // hold left bumper for orbit mode (face and orbit around target)
    driver.leftBumper()
        .whileTrue(swerve.orbitCommand(driver));

    // hold right bumper to aim at hub (auto-rotate to face hub, driver translates)
    driver.rightBumper()
        .whileTrue(swerve.aimAtHubCommand(driver));

    // Y button: drive to shooting distance and shoot
    driver.y().onTrue(ShootingCommands.driveAndShootCommand(swerve, launcher));

    // zero pose on left click, accept vision pose on right click
    driver.leftStick().onTrue(swerve.zeroPoseCommand());
    driver.rightStick().onTrue(limelight.resetPoseFromVisionCommand());
  }

  public Command getAutonomousCommand() {
    return auto.generateCommand();
  }
}

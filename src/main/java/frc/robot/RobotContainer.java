// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.led.LEDHardwareBlinkin;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.CommandLogger;
import frc.robot.util.Field;
import frc.robot.util.Util;

public class RobotContainer {

    static final int LED_PWM_PORT = 0;
    static final int INTAKE_CAN_ID = 9;
    static final int FEEDER_CAN_ID = 32;
    static final int AGITATOR_CAN_ID = 11;
    static final int SHOOTER_CAN_ID = 60;

    final GameController driver = new GameController(0);
    final GameController operator = new GameController(1);

    // core subsystems
    final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    final SwerveSubsystem swerve = new SwerveSubsystem(drivetrain);
    final LimelightSubsystem limelight = new LimelightSubsystem(swerve);
    final AutonomousSubsystem auto;

    // launcher + intake + LED
    final LauncherSubsystem launcher = new LauncherSubsystem(Robot.isSimulation()
                ? new LauncherHardwareSim()
                : new LauncherHardwareRev(INTAKE_CAN_ID, FEEDER_CAN_ID, AGITATOR_CAN_ID, SHOOTER_CAN_ID));
    final LEDSubsystem led = new LEDSubsystem(Robot.isSimulation()
                ? new LEDHardwareSim() 
                : new LEDHardwareBlinkin(LED_PWM_PORT));

    public RobotContainer() {

        CommandLogger.enable();
        CommandLogger.addController("Driver", driver);
        CommandLogger.addController("Operator", operator);

        auto = new AutonomousSubsystem(swerve);

        // default commands
        swerve.setDefaultCommand(swerve.driveCommand(driver));
        launcher.setDefaultCommand(launcher.coast());

        // LED distance to hub
        led.setDistanceSupplier(() -> Util.feetBetween(swerve.getPose(), Field.getHubCenter()));

        configureBindings();
    }

    public Command getAutonomousCommand() {
        return auto.generateCommand();
    }

    private void configureBindings() {

        // driver bindings
        // driver.leftBumper().whileTrue(swerve.orbitCommand(driver));
        driver.rightBumper().whileTrue(ShootingCommands.aimAtHubCommand(swerve));
        driver.y().onTrue(ShootingCommands.driveAndShootCommand(swerve, launcher));
        driver.leftTrigger().whileTrue(ShootingCommands.jiggleCommand(swerve));
        driver.leftStick().onTrue(swerve.zeroPoseCommand());
        driver.rightStick().onTrue(limelight.resetPoseFromVisionCommand());

        // operator bindings
        operator.a().whileTrue(launcher.intakeCommand());
        operator.b().whileTrue(launcher.shootCommand());
    }
}

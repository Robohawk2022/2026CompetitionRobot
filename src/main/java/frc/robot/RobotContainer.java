// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.led.LEDHardwareBlinkin;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;

public class RobotContainer {

    public static final int LED_PWM_PORT = 0;
    public static final int INTAKE_CAN_ID = 60;
    public static final int FEEDER_CAN_ID = 9;
    public static final int AGITATOR_CAN_ID = 2;
    public static final int SHOOTER_CAN_ID = 35;

    final GameController driver;
    final SwerveSubsystem swerve;
    final LimelightSubsystem limelight;
    final AutonomousSubsystem auto;
    final LauncherSubsystem launcher;
    final LEDSubsystem led;

    public RobotContainer() {

        driver = new GameController(0);

        // swerve drive (default command is driving in teleop mode)
        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        swerve.setDefaultCommand(swerve.driveCommand(driver));

        limelight = new LimelightSubsystem(swerve);
        auto = new AutonomousSubsystem(swerve);

        // launcher (default command is coasting the wheels)
        launcher = new LauncherSubsystem(RobotBase.isSimulation()
                ? new LauncherHardwareSim()
                : new LauncherHardwareRev(
                        INTAKE_CAN_ID,
                        FEEDER_CAN_ID,
                        AGITATOR_CAN_ID,
                        SHOOTER_CAN_ID));
        launcher.setDefaultCommand(launcher.coast());

        // LED
        //  - default command is showing range to shooter
        //  - as long as odometry is broken, flash the error signal
        led = new LEDSubsystem(RobotBase.isSimulation()
                ? new LEDHardwareSim()
                : new LEDHardwareBlinkin(LED_PWM_PORT));
        led.setDefaultCommand(ShootingCommands.flashWhenSHootable(swerve, led));
        limelight.odometryBrokenTrigger().whileTrue(led.flash(LEDSignal.ERROR));

        configureBindings();
    }

    public Command getAutonomousCommand() {
        return auto.generateCommand();
    }

    private void configureBindings() {

        // sticks and left/right trigger are already taken by swerve teleop

        // a/b/x/y are "pure" shooting commands
        driver.a().onTrue(launcher.intakeCommand());
        driver.b().onTrue(launcher.shootCommand());
        driver.x().onTrue(launcher.ejectCommand());
        driver.y().onTrue(launcher.coast());

        // bumpers exercise auto shooting
        driver.leftBumper().whileTrue(ShootingCommands.orientToShoot(swerve));
        driver.rightBumper().whileTrue(ShootingCommands.driveAndShootCommand(swerve, launcher));

        // sticks reset pose
        driver.leftStick().onTrue(swerve.zeroPoseCommand());
        driver.rightStick().onTrue(limelight.resetPoseFromVisionCommand());

    }
}

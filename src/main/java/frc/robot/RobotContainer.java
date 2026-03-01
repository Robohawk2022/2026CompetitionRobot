// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.ballpath.BallPathHardwareRev;
import frc.robot.subsystems.ballpath.BallPathHardwareSim;
import frc.robot.subsystems.ballpath.BallPathSubsystem;
import frc.robot.subsystems.led.LEDHardwareBlinkin;
import frc.robot.subsystems.led.LEDHardwareSim;
import frc.robot.subsystems.led.LEDSignal;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterHardwareRev;
import frc.robot.subsystems.shooter.ShooterHardwareSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.Field;

public class RobotContainer {

    public static boolean RESET_PREFS = true;

    public static final int LED_PWM_PORT = 0;
    public static final int INTAKE_CAN_ID = 60;
    public static final int FEEDER_CAN_ID = 9;
    public static final int AGITATOR_CAN_ID = 2;
    public static final int SHOOTER_CAN_ID = 35;

    final GameController driver;
    final SwerveSubsystem swerve;
    final LimelightSubsystem limelight;
    final AutonomousSubsystem auto;
    final ShooterSubsystem shooter;
    final BallPathSubsystem ballPath;
    final LEDSubsystem led;

    public RobotContainer() {

        if (RESET_PREFS) {
            Preferences.removeAll();
        }

        driver = new GameController(0);

        // swerve drive (default command is driving in teleop mode)
        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        swerve.setDefaultCommand(swerve.driveCommand(driver));

        limelight = new LimelightSubsystem(swerve);
        auto = new AutonomousSubsystem(swerve);

        // shooter (default command is coasting)
        shooter = new ShooterSubsystem(RobotBase.isSimulation()
                ? new ShooterHardwareSim()
                : new ShooterHardwareRev(SHOOTER_CAN_ID));
        shooter.setDefaultCommand(shooter.coast());

        // ball path (default command is coasting)
        ballPath = new BallPathSubsystem(RobotBase.isSimulation()
                ? new BallPathHardwareSim()
                : new BallPathHardwareRev(
                        INTAKE_CAN_ID,
                        FEEDER_CAN_ID,
                        AGITATOR_CAN_ID));
        ballPath.setDefaultCommand(ballPath.coast());

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

        // a/b/x/y are ball-handling commands
        driver.a().whileTrue(ShootingCommands.intakeMode(ballPath, shooter));
        driver.b().whileTrue(shooter.intakeCommand());
        driver.x().whileTrue(ShootingCommands.shootMode(ballPath, shooter));
        driver.y().onTrue(Commands.parallel(shooter.coast(), ballPath.coast()));

        // bumpers exercise auto shooting
//        driver.leftBumper().whileTrue(ShootingCommands.orientToShoot(swerve));
//        driver.rightBumper().whileTrue(ShootingCommands.driveAndShootCommand(swerve, shooter, ballPath));
        driver.rightBumper().whileTrue(ShootingCommands.orbitCommand(
                driver,
                swerve,
                Pose2d.kZero));

        // sticks reset pose
        driver.leftStick().onTrue(swerve.zeroPoseCommand());
        driver.rightStick().onTrue(limelight.resetPoseFromVisionCommand());

    }
}

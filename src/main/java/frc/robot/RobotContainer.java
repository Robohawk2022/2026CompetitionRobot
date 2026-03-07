// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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

public class RobotContainer {

    // flip this to reset all preferences once the robot starts up
    public static boolean RESET_PREFS = false;

    public static final int LED_PWM_PORT = 0;
    public static final int INTAKE_CAN_ID = 8;
    public static final int FEEDER_CAN_ID = 9;
    public static final int AGITATOR_CAN_ID = 2;
    public static final int SHOOTER_CAN_ID = 35;

    public final GameController driver;
    public final SwerveSubsystem swerve;
    public final LimelightSubsystem limelight;
    public final AutonomousSubsystem auto;
    public final ShooterSubsystem shooter;
    public final BallPathSubsystem ballPath;
    public final LEDSubsystem led;

    public RobotContainer() {

        if (RESET_PREFS) {
            Preferences.removeAll();
        }

        driver = new GameController(0);

        // swerve drive (default command is driving in teleop mode)
        swerve = new SwerveSubsystem(TunerConstants.createDrivetrain());
        swerve.setDefaultCommand(swerve.teleopCommand(driver));

        limelight = new LimelightSubsystem(swerve);

        // shooter (default command is coasting)
        shooter = new ShooterSubsystem(RobotBase.isSimulation()
                ? new ShooterHardwareSim()
                : new ShooterHardwareRev(SHOOTER_CAN_ID),
                driver.y());
        shooter.setDefaultCommand(shooter.idleCommand());

        // ball path (default command is coasting)
        ballPath = new BallPathSubsystem(RobotBase.isSimulation()
                ? new BallPathHardwareSim()
                : new BallPathHardwareRev(
                        INTAKE_CAN_ID,
                        FEEDER_CAN_ID,
                        AGITATOR_CAN_ID));
        ballPath.setDefaultCommand(ballPath.coast());

        // LED
        //  - default command will flash when odometry drifts
        led = new LEDSubsystem(RobotBase.isSimulation()
                ? new LEDHardwareSim()
                : new LEDHardwareBlinkin(LED_PWM_PORT));
        led.setDefaultCommand(led.show(() -> idleLedSignalCalculator(swerve, limelight)));

        auto = new AutonomousSubsystem(
                swerve,
                shooter,
                ballPath,
                led);

        configureBindings();
    }

    /**
     * @return default signal for LED subsystem
     */
    public static LEDSignal idleLedSignalCalculator(SwerveSubsystem swerve, LimelightSubsystem limelight) {
        if (ShootingCommands.isInShootingPosition(swerve)) {
            return LEDSignal.SHOOTABLE;
        } else if (limelight.isPoseResetRecommended()) {
            return LEDSignal.POSE_RESET;
        } else {
            return LEDSignal.IDLE;
        }
    }

    public Command getAutonomousCommand() {
       return auto.generateCommand();
        // return Commands.print("skipping auto (for now)");
    }

    private void configureBindings() {

        // sticks and left/right trigger are already taken by swerve teleop

        // a/b/x/y are ball-handling commands
        driver.leftTrigger().whileTrue(ShootingCommands.intakeMode(led, ballPath, shooter));
        driver.rightTrigger().whileTrue(ShootingCommands.shootMode(led, ballPath, shooter));
        driver.b().whileTrue(ballPath.ejectCommand());
        driver.x().whileTrue(ShootingCommands.jiggle(swerve));

        driver.start().whileTrue(swerve.driveToHeadingCommand(Rotation2d.k180deg));

        // sticks reset pose
        driver.povUp().onTrue(swerve.zeroPoseCommand());
        driver.povDown().onTrue(limelight.resetPoseFromVisionCommand());
        driver.back().whileTrue(ShootingCommands.orientToShoot(led, swerve));

        // driver.start().onTrue(ShootingCommands.openHopper(swerve));
       // driver.start().onTrue(getAutonomousCommand());
    }
}

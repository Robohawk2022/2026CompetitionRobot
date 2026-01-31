package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.hubberdshooter.HubberdShooterHardwareCTRE;
import frc.robot.subsystems.hubberdshooter.HubberdShooterHardwareSim;
import frc.robot.subsystems.hubberdshooter.HubberdShooterSubsystem;

/**
 * Standalone test program for the HubberdShooter subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=HubberdShooterTestbot}
 * <p>
 * Uses {@link GameController} which auto-maps buttons for Xbox, 8BitDo, and Logitech.
 * Set {@code SwerveTeleop/UseXboxMapping?} to {@code false} for 8BitDo controllers.
 * <p>
 * Button mappings:
 * <ul>
 *   <li>A = Intake</li>
 *   <li>B = Outtake</li>
 *   <li>X = Shoot</li>
 *   <li>Y = Stop</li>
 *   <li>Left Bumper = Test Motor 1 only</li>
 *   <li>Right Bumper = Test Motor 2 only</li>
 * </ul>
 */
public class HubberdShooterTestbot extends TimedRobot {

    private HubberdShooterSubsystem shooter;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> HubberdShooterTestbot starting...");

        shooter = new HubberdShooterSubsystem(
                isSimulation() ? new HubberdShooterHardwareSim() : new HubberdShooterHardwareCTRE());
        controller = new GameController(0);

        // Set default command to idle
        shooter.setDefaultCommand(shooter.idleCommand());

        // Button bindings - uses GameController which handles Xbox/8BitDo/Logitech
        controller.a().whileTrue(shooter.intakeCommand());
        controller.b().whileTrue(shooter.outtakeCommand());
        controller.x().whileTrue(shooter.shootCommand());
        controller.y().onTrue(shooter.stopCommand());
        controller.leftBumper().whileTrue(shooter.testMotor1Command());
        controller.rightBumper().whileTrue(shooter.testMotor2Command());

        System.out.println(">>> Controller type: " + controller.getType());
        System.out.println(">>> Button mappings:");
        System.out.println("    A = Intake");
        System.out.println("    B = Outtake");
        System.out.println("    X = Shoot");
        System.out.println("    Y = Stop");
        System.out.println("    LB = Test Motor 1 ONLY");
        System.out.println("    RB = Test Motor 2 ONLY");
        System.out.println("");
        System.out.println(">>> If buttons are wrong, check SwerveTeleop/UseXboxMapping? preference");
        System.out.println("    true = Xbox controller");
        System.out.println("    false = 8BitDo controller");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Show controller type for debugging
        SmartDashboard.putString("ControllerType", controller.getType().toString());
    }

    @Override
    public void disabledInit() {
        shooter.stop();
    }
}

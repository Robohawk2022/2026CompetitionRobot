package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.hubbardshooter.HubbardShooterHardwareRev;
import frc.robot.subsystems.hubbardshooter.HubbardShooterHardwareSim;
import frc.robot.subsystems.hubbardshooter.HubbardShooterSubsystem;

/**
 * Standalone test program for the HubbardShooter subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=HubbardShooterTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake (both motors pull fuel in)</li>
 *   <li>B (hold) - Outtake (both motors push fuel out)</li>
 *   <li>X (hold) - Shoot (motors counter-rotate)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 */
public class HubbardShooterTestbot extends TimedRobot {

    private HubbardShooterSubsystem shooter;
    private GameController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> HubbardShooterTestbot starting...");

        boolean sim = isSimulation();
        shooter = new HubbardShooterSubsystem(
                sim ? new HubbardShooterHardwareSim() : new HubbardShooterHardwareRev());
        controller = new GameController(0);

        shooter.setDefaultCommand(shooter.idleCommand());

        controller.a().whileTrue(shooter.intakeCommand());
        controller.b().whileTrue(shooter.outtakeCommand());
        controller.x().whileTrue(shooter.shootCommand());
        controller.y().onTrue(shooter.stopCommand());

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake");
        System.out.println("    B (hold) = Outtake");
        System.out.println("    X (hold) = Shoot (counter-rotate)");
        System.out.println("    Y (press) = Stop");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        shooter.stop();
    }
}

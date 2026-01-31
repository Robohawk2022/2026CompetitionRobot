package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.hubberdshooter.HubberdShooterHardwareCTRE;
import frc.robot.subsystems.hubberdshooter.HubberdShooterHardwareSim;
import frc.robot.subsystems.hubberdshooter.HubberdShooterSubsystem;

/**
 * Standalone test program for the HubberdShooter subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=HubberdShooterTestbot}
 * <p>
 * Button mappings for 8BitDo controller:
 * <ul>
 *   <li>A (button 1) = Intake</li>
 *   <li>B (button 2) = Outtake</li>
 *   <li>X (button 4) = Shoot</li>
 *   <li>Y (button 5) = Stop</li>
 *   <li>Left Bumper (button 7) = Test Motor 1 only</li>
 *   <li>Right Bumper (button 8) = Test Motor 2 only</li>
 * </ul>
 */
public class HubberdShooterTestbot extends TimedRobot {

    private HubberdShooterSubsystem shooter;
    private CommandXboxController controller;

    @Override
    public void robotInit() {
        System.out.println(">>> HubberdShooterTestbot starting...");

        shooter = new HubberdShooterSubsystem(
                isSimulation() ? new HubberdShooterHardwareSim() : new HubberdShooterHardwareCTRE());
        controller = new CommandXboxController(0);

        // Set default command to idle
        shooter.setDefaultCommand(shooter.idleCommand());

        // Button bindings - 8BitDo raw button numbers
        controller.button(1).whileTrue(shooter.intakeCommand());       // A
        controller.button(2).whileTrue(shooter.outtakeCommand());      // B
        controller.button(4).whileTrue(shooter.shootCommand());        // X
        controller.button(5).onTrue(shooter.stopCommand());            // Y
        controller.button(7).whileTrue(shooter.testMotor1Command());   // Left Bumper
        controller.button(8).whileTrue(shooter.testMotor2Command());   // Right Bumper

        System.out.println(">>> Button mappings (8BitDo):");
        System.out.println("    A (btn 1) = Intake");
        System.out.println("    B (btn 2) = Outtake");
        System.out.println("    X (btn 4) = Shoot");
        System.out.println("    Y (btn 5) = Stop");
        System.out.println("    LB (btn 7) = Test Motor 1 ONLY");
        System.out.println("    RB (btn 8) = Test Motor 2 ONLY");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Show which raw buttons are pressed (for debugging)
        StringBuilder pressed = new StringBuilder();
        for (int i = 1; i <= 15; i++) {
            if (controller.getHID().getRawButton(i)) {
                if (pressed.length() > 0) pressed.append(", ");
                pressed.append(i);
            }
        }
        SmartDashboard.putString("RawButtonsPressed", pressed.length() > 0 ? pressed.toString() : "none");
    }

    @Override
    public void disabledInit() {
        shooter.stop();
    }
}

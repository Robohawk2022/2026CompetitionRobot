package frc.robot.testbots;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GameController;
import frc.robot.subsystems.launcher.LauncherHardwareRev;
import frc.robot.subsystems.launcher.LauncherHardwareSim;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Standalone test program for the Launcher subsystem (4-motor design).
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=LauncherTestbot}
 * <p>
 * <b>Button mappings:</b>
 * <ul>
 *   <li>A (hold) - Intake (feeders spin inward)</li>
 *   <li>B (hold) - Eject (feeders spin outward)</li>
 *   <li>X (hold) - Shoot (feeders + shooter spin up)</li>
 *   <li>Y (press) - Stop all motors</li>
 * </ul>
 */
public class LauncherTestbot extends TimedRobot {

    // CAN IDs from the testing setup
    public static final int INTAKE_CAN_ID = 9;
    public static final int FEEDER_CAN_ID = 60;
    public static final int AGITATOR_CAN_ID = 2;
    public static final int SHOOTER_CAN_ID = 32;

    private LauncherSubsystem launcher;
    private double testRpm;

    @Override
    public void robotInit() {

        testRpm = 1000.0;

        System.out.println(">>> LauncherTestbot starting...");

        // clear stale Preferences so code defaults always win on deploy
        Preferences.removeAll();

        launcher = new LauncherSubsystem(isSimulation()
                ? new LauncherHardwareSim()
                : new LauncherHardwareRev(INTAKE_CAN_ID, FEEDER_CAN_ID, AGITATOR_CAN_ID, SHOOTER_CAN_ID));

        GameController controller = new GameController(0);

        launcher.setDefaultCommand(launcher.coast());

        // intake: feeders spin inward
        controller.a().whileTrue(launcher.intakeCommand());

        // eject: feeders spin outward
        controller.b().whileTrue(launcher.ejectCommand());

        // shoot: feeders + shooter spin up
        controller.x().whileTrue(launcher.shootCommand());

        // left bumper: run all motors at test velocity
        controller.leftBumper().whileTrue(launcher.defer(() ->
                launcher.velocityCommand(testRpm, testRpm, testRpm, testRpm)));

        System.out.println(">>> Button mappings:");
        System.out.println("    A (hold) = Intake (feeders inward)");
        System.out.println("    B (hold) = Eject (feeders outward)");
        System.out.println("    X (hold) = Shoot (feeders + shooter)");

        SmartDashboard.putData("LauncherTestbot", builder -> {
            builder.addDoubleProperty("TestRpm", () -> testRpm, val -> testRpm = val);
        });

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        launcher.coast();
    }
}

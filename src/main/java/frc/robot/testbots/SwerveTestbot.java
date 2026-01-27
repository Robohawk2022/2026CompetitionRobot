package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameController;
import frc.robot.subsystems.swerve.SwerveHardwareCTRE;
import frc.robot.subsystems.swerve.SwerveHardwareSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Standalone test program for the swerve drive subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=SwerveTestbot}
 */
public class SwerveTestbot extends TimedRobot {

    private SwerveSubsystem swerve;
    private GameController controller;

    // track last 2 buttons pressed
    private String lastButton = "(none)";
    private String previousButton = "(none)";

    @Override
    public void robotInit() {
        System.out.println(">>> SwerveTestbot starting...");

        // use appropriate hardware based on environment
        swerve = new SwerveSubsystem(
                isSimulation() ? new SwerveHardwareSim() : new SwerveHardwareCTRE());
        controller = new GameController(0);

        // reset wheels to forward facing on startup
        swerve.resetWheelsCommand().schedule();

        System.out.println(">>> Button bindings configured - press A, B, X, Y, Start, Back, etc.");

        // default to field-relative driving with turbo/sniper modes
        swerve.setDefaultCommand(swerve.driveCommand(controller));

        // Odometry reset when both thumbsticks are clicked
        controller.leftStick()
                .and(controller.rightStick())
                .onTrue(logButton("BothSticks").andThen(swerve.resetPoseCommand()));

        // Orbit mode: hold left bumper to orbit around the Reef while facing it
        controller.leftBumper()
                .onTrue(logButton("LB-Orbit"))
                .whileTrue(swerve.orbitCommand(controller));

        // publish button tracking to Shuffleboard
        SmartDashboard.putData("ButtonLog", builder -> {
            builder.addStringProperty("Last", () -> lastButton, null);
            builder.addStringProperty("Previous", () -> previousButton, null);
        });

        // debug controller inputs
        SmartDashboard.putData("Controller", builder -> {
            builder.addBooleanProperty("LeftBumper", () -> controller.getHID().getRawButton(5), null);
            builder.addBooleanProperty("RightBumper", () -> controller.getHID().getRawButton(6), null);
            builder.addDoubleProperty("LeftX", controller::getLeftX, null);
            builder.addDoubleProperty("LeftY", controller::getLeftY, null);
            builder.addDoubleProperty("RightX", controller::getRightX, null);
            builder.addDoubleProperty("LeftTrigger", controller::getLeftTriggerAxis, null);
            builder.addDoubleProperty("RightTrigger", controller::getRightTriggerAxis, null);
        });
    }

    private edu.wpi.first.wpilibj2.command.Command logButton(String name) {
        return Commands.runOnce(() -> {
            previousButton = lastButton;
            lastButton = name;
            System.out.println(">>> BUTTON PRESSED: " + name);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putString("LastButton", lastButton);
        SmartDashboard.putString("PreviousButton", previousButton);
    }

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {}
}

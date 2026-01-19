package frc.robot.testbots;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveHardwareCTRE;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Standalone test program for the swerve drive subsystem.
 * <p>
 * Run with: {@code ./gradlew simulateJava -Probot=SwerveTestbot}
 */
public class SwerveTestbot extends TimedRobot {

    private static final double DEADZONE = 0.27;

    private SwerveSubsystem swerve;
    private CommandXboxController controller;

    // track last 2 buttons pressed
    private String lastButton = "(none)";
    private String previousButton = "(none)";

    @Override
    public void robotInit() {
        System.out.println(">>> SwerveTestbot starting...");

        // use simulation hardware
        // swerve = new SwerveSubsystem(new SwerveSim());
        swerve = new SwerveSubsystem(new SwerveHardwareCTRE());
        controller = new CommandXboxController(0);

        System.out.println(">>> Button bindings configured - press A, B, X, Y, Start, Back, etc.");

        // default to field-relative driving
        swerve.setDefaultCommand(swerve.driveCommand(
                () -> -MathUtil.applyDeadband(controller.getLeftY(), DEADZONE),
                () -> -MathUtil.applyDeadband(controller.getLeftX(), DEADZONE),
                () -> -MathUtil.applyDeadband(controller.getRightX(), DEADZONE),
                true));

        // // button bindings with logging
        // controller.start().onTrue(logButton("Start").andThen(swerve.zeroHeadingCommand()));
        // controller.back().onTrue(logButton("Back").andThen(swerve.resetPoseCommand()));
        // controller.x().onTrue(logButton("X")).whileTrue(swerve.idleCommand());
        // controller.y().onTrue(logButton("Y"));
        // controller.a().onTrue(logButton("A"));
        // controller.b().onTrue(logButton("B"));
        // controller.leftBumper().onTrue(logButton("LB"));
        // controller.rightBumper().onTrue(logButton("RB"));
        // controller.leftTrigger().onTrue(logButton("LT"));
        // controller.rightTrigger().onTrue(logButton("RT"));
        // controller.leftStick().onTrue(logButton("LeftStick"));
        // controller.rightStick().onTrue(logButton("RightStick"));
        // controller.pov(0).onTrue(logButton("DPad-Up"));
        // controller.pov(90).onTrue(logButton("DPad-Right"));
        // controller.pov(180).onTrue(logButton("DPad-Down"));
        // controller.pov(270).onTrue(logButton("DPad-Left"));

        // Odometry reset when both thumbsticks are clicked
        controller.leftStick()
                .and(controller.rightStick())
                .onTrue(logButton("BothSticks").andThen(swerve.resetPoseCommand()));

        // publish button tracking to Shuffleboard
        SmartDashboard.putData("ButtonLog", builder -> {
            builder.addStringProperty("Last", () -> lastButton, null);
            builder.addStringProperty("Previous", () -> previousButton, null);
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

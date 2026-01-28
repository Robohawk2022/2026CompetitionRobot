package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.DigitBoardProgramPicker;
import frc.robot.util.Util;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiConsumer;

import static frc.robot.Config.PathPlanner.debugLogging;
import static frc.robot.Config.PathPlanner.rotationD;
import static frc.robot.Config.PathPlanner.rotationI;
import static frc.robot.Config.PathPlanner.rotationP;
import static frc.robot.Config.PathPlanner.translationD;
import static frc.robot.Config.PathPlanner.translationI;
import static frc.robot.Config.PathPlanner.translationP;

/**
 * Subsystem that manages a set of declared autonomous programs and allows
 * selecting one to run.
 */
public class AutonomousSubsystem extends SubsystemBase {

//region Implementation --------------------------------------------------------

    final SwerveSubsystem swerve;
    final DigitBoardProgramPicker picker;
    String selected;
    Command command;
    boolean error;

    public AutonomousSubsystem(SwerveSubsystem swerve) {

        this.swerve = Objects.requireNonNull(swerve);
        this.picker = new DigitBoardProgramPicker(
                "Auto Program",
                getProgramNames());
        this.selected = picker.get();

        // there's a lot happening under the covers here, and PathPlanner
        // might generate an error if there's a problem with config files,
        // or calculating a path, or something like that.
        //
        // we don't want the entire robot to crash if there's a problem
        // with autonomous, so we wrap this whole thing with an exception
        // handler. if there is an error, we will log it and then use our
        // emergency backup configuration

        try {
            configureAutoBuilder();
            this.error = false;
        } catch (Exception e) {
            this.error = true;
        }

        if (debugLogging.getAsBoolean()) {
            configureLogging();
        }

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Program", () -> selected, null);
            builder.addBooleanProperty("Running?", () -> command != null && command.isScheduled(), null);
            builder.addBooleanProperty("Error?", () -> error, null);
        });
    }

    /**
     * Configure PathPlanner's command builder
     */
    private void configureAutoBuilder() throws Exception {

        // this loads the settings for the robot, which you edited and
        // saved through the GUI
        Util.log("[auto] loading robot configuration");
        RobotConfig config = RobotConfig.fromGUISettings();

        // this registers all the commands we'll use when executing our
        // selected program
        registerNamedCommands();

        // this is what accepts calculated speeds from the path engine
        // and actually applies them to drive the robot around; if the
        // robot isn't moving at all, start debugging here
        BiConsumer<ChassisSpeeds,DriveFeedforwards> output = (s, f) -> {
            SmartDashboard.putNumber("PP_X", s.vxMetersPerSecond);
            SmartDashboard.putNumber("PP_Y", s.vyMetersPerSecond);
            swerve.driveRobotRelative("path-planner", s);
        };

        // this is used to provide feedback to keep the robot on the
        // supplied course; if the robot isn't moving accurately, start
        // debugging here
        PathFollowingController controller = new PPHolonomicDriveController(
                new PIDConstants(translationP.getAsDouble(), translationI.getAsDouble(), translationD.getAsDouble()),
                new PIDConstants(rotationP.getAsDouble(), rotationI.getAsDouble(), rotationD.getAsDouble())
        );

        Util.log("[auto] configuring AutoBuilder");
        AutoBuilder.configure(
                swerve::getPose,
                swerve::resetPose,
                swerve::getCurrentSpeeds,
                output,
                controller,
                config,

                // PP plans are drawn from the perspective of the blue team;
                // if we are the red team, PP should flip the path for us
                Util::isRedAlliance,

                this);
    }

    /**
     * Configures logging for PathPlanner. This spams out a LOT of data, so
     * be thoughtful when using it.
     */
    private void configureLogging() {
        PathPlannerLogging.setLogTargetPoseCallback(pose -> {
            Util.log("[auto] target pose: %s", pose);
        });
        PathPlannerLogging.setLogActivePathCallback(path -> {
            Util.log("[auto] active path: %s", path);
        });
        PathPlannerLogging.setLogCurrentPoseCallback(pose -> {
            Util.log("[auto] current pose: %s", pose);
        });
    }

    @Override
    public void periodic() {
        selected = picker.get();
    }

    /**
     * Call this from {@link TimedRobot#autonomousInit()} to create the
     * actual command. We do all the work of loading the program and
     * configuring PathPlanner here, so the field operator doesn't have
     * to wait for it to happen.
     */
    public Command generateCommand() {

        if (selected == null) {
            Util.log("[auto] NO SELECTED PROGRAM!!!");
            return Commands.none();
        }

        // if initialization failed, we will use our emergency backup
        // command instead of trying to do a full routine
        if (error) {
            command = createEmergencyCommand();
        }

        else {

            // this loads the actual program description and links it
            // up with all the other configurations
            command = new PathPlannerAuto(selected);

            // in years past we might mess with the program once it was
            // loaded - for instance, prepending some initialization code
            // or adding some timeouts or other such hackery
            command = decorateAutoCommand(command);
        }

        return command;
    }

//endregion

//region Customization ---------------------------------------------------------

    /*
     * TODO identify all your programs
     *
     * This is the list of all the autonomous programs we have to choose
     * from.
     *
     * There should be one entry for each different program. The value on
     * the left is a "short name" (which gets displayed on the DigitBoard)
     * and the value on the right is the actual filename of the program
     * that you edited using PathPlanner.
     */
    private Map<String,String> getProgramNames() {

        // TODO we can get names from AutoBuilder - can we auto-generate codes?

        // we use a LinkedHashMap so the programs will be shown in the
        // same order as below
        Map<String,String> programs = new LinkedHashMap<>();
        programs.put("EX01", "Example1");
        programs.put("EX02", "Example2");
        programs.put("EX03", "Example3");
        return programs;
    }

    /*
     * TODO create all your named commands
     *
     * This is where you register all the "named commands" that are used
     * by your different autonomous programs
     */
    private void registerNamedCommands() {

        Util.log("[auto] Registering named commands");

        // you will have to create commands that do real things like
        // shooting and scoring if you want to use them in your autos
        NamedCommands.registerCommand("DoSomething1", Commands.print("*** thing 1 ***"));
        NamedCommands.registerCommand("DoSomething2", Commands.print("*** thing 2 ***"));
        NamedCommands.registerCommand("DoSomething3", Commands.print("*** thing 3 ***"));
    }

    /*
     * TODO do you need to "decorate" your auto program?
     *
     * In previous years we've had to do some "mandatory" startup tasks,
     * like lowering an arm or moving the position of a held gamepiece.
     * We did this by taking the Command created by PathPlanner and adding
     * stuff to the beginning or end.
     */
    private Command decorateAutoCommand(Command autoCommand) {
        Command printAlliance = Commands.runOnce(() -> {
            String alliance = Util.isRedAlliance() ? "RED" : "BLUE";
            Util.log("[auto] starting auto as %s alliance", alliance);
        });
        Command logComplete = Commands.print("[auto] done with auto");
        return printAlliance.andThen(autoCommand).andThen(logComplete);
    }

    /*
     * TODO create an emergency auto program
     *
     * If the autonomous routine gets buggered up, what do you want the
     * robot to do? Sitting still might be one option, but you mightz also
     * have some of that "mandatory" stuff to do. Or you might want to try
     * some minimal driving to score points.
     */
    private Command createEmergencyCommand() {
        return Commands.print("[auto] oh, dang, something went wrong!");
    }

    /*
     * TODO create an "emergency" start pose
     *
     * The robot gets its starting pose from the autonomous path. If there
     * isn't one, what should you do? You probably at least want to consider
     * guessing the heading of the robot, so the driver has some little bit
     * of control and can e.g. get to an AprilTag and reset position from
     * there using the Limelight.
     */
    private Pose2d createEmergencyStartPose() {

        // example: assume the robot starts facing away from
        // the alliance wall (blue is 0, red is 180)
        return Util.isRedAlliance()
                ? new Pose2d(0.0, 0.0, Rotation2d.k180deg)
                : Pose2d.kZero;
    }
}

# TEST PLAN

* Test shooter spin up
    * On teleop enable, it shouldn't move
    * Hold left trigger
        * LED should flash yellow
        * Should go to intake speed
        * LED should show solid green
    * Release left trigger
        * LED should go back to pink
        * Should remain at intake speed
    * Hold right trigger
        * LED should flash yellow
        * Should go to shoot speed
        * LED should show rainbow party
    * Release left trigger
        * LED should go back to pink
        * Should go back to intake speed
    * Disable teleop
        * Wheel should spin down to 0
    * Re-enable teleop
        * It shouldn't move (rinse and repeat)

* Prep for auto testing
    * Confirm all measurements
        * Weight and module position measurements in SwerveHardwareConfig
        * Weight, module position and bumper measurements in PathPlanner
    * Visually inspect paths with new measurements

* Run auto
    * Review AutonomousSubsystem.getProgramNames to make sure they are registered
    * Review AutonomousSubsystem.registerNamedCommands to make sure required commands exist
    * Pick one & run it
    * Watch for commands in console
    * Watch pose in AdvantageScope

* Test shooting LED signal
    * With a good pose ... 
    * Drive into a shooting position
    * LED should show green
    * Check to see how "fiddly" it is and tune
        * BallHandling/ShootDistance
        * BallHandling/ShootDistanceTolerance
        * BallHandling/ShootAngleTolerance

* Test positioning
    * Update controller mapping to include ShootingCommands.orientToShoot
    * With a good pose ...
    * Drive close to the hub
    * Trigger the command and see if the robot drives to shooting position
    * Once it stops, LEDe should show solid green

* Test vision
    * Enable Limelight
    * Drive to where you can see a tag
    * Use NetworkTables & AdvantageScope to visualize
        * MegaTag1-RobotPose
        * MegaTag2-RobotPose
            * Should be near robot's current pose on the field 
        * MegaTag1-TagPose
        * MegaTag2-TagPose
            * Should show you where the tag is  
    * Click to reset from vision pose
        * Watch the robot's position jump in AdvantageScope
        * Drive around and luxuriate

# NOTES ON TESTING AUTO

## Robot measurements & configuration

We need to verify a few measurements and make sure they are registered in
the right places. In general:

* The Swerve Drive cares about the gearing on the wheels, current limits for
motors, and the position of the wheels relative to the center of the robot.
That stuff is all in `SwerveHardwareConfig`.

* The PathPlanner library (which runs the autonomous routine during the match)
is configured in `AutonomousSubsystem`. It reuses some of those configuration
properties. It *also* cares about stuff like the robot's weight and moment
of inertia (MOI); for simplicity we store that stuff in `SwerveHardwareConfig`
as well.

* The PathPlanner GUI wants to know that stuff PLUS information about the
geometry of the bumpers, so it can accurately draw the robot on the field.
That information is stored in "Robot Config" in the application's settings.

* PathPlanner GUI also has some "speed limits" under "App Settings". It
lets you specify a maximum translation and rotation velocity and acceleration.

## Common gotchas

* The GUI is a little buggy with "linked waypoints". I notice that sometimes
they will get reset. I usually make it a habit to visually inspect the 
autonomous routines and make sure they "look right" before deploying to the
robot if I'm going to run autos.

* There's also nothing that will prevent you from naming a command in the
GUI that isn't registered in the `AutonomousSubsystem`. Make sure all the
commands you want to use are registered there - deployment is a good time
to check that, as well.

* Before you run your auto program, make sure you know which alliance
you think you are, and which one the Driver Station thinks you are. (You
probably always want to run as blue for now.) The impact could be wonky 
pose reports in the dashboard.

* If the robot doesn't move at all, something is wrong. Make sure your 
program is actually running (the `AutonomousSubsystem` will log the start
and end of the program to the console).

* Another thing that can prevent the robot from running is if a Command
accidentally runs forever. If the robot seems frozen, make sure you don't
have a runaway command. You can always add timeouts to Commands as you
register them.

* If the robot doesn't accurately follow the path, you may need tuning.
There are separate kP parameters for translation and rotation when 
running an auto path. They are declared in `Config`:
  * `PathPlanner/Translation/kP`
  * `PathPlanner/Rotation/kP`

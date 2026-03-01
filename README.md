# TEST PLAN

[x] Tune the new shooter wheel
    * Holding B will run shooter wheel only
    * Speeds in ShooterSubsystem
    * Look for "at speed" on the dashboard

[x] Test swerve modes
    * Left trigger = sniper
    * Right trigger = turbo

* Test swerve orbit
    * Right bumper = orbit starting position
        * Will rotate to face position
        * Right stick = spin around
        * Left stick = forward/back
        * Left stick button zeros pose

[x] Test opening the hopper
    * Prefs are in SwerveAuto 
    * Start button runs routine     

[x] Test jiggle
    * Y will jiggle 

[x] Test intake mode
    * Holding A will run in take mode
    * Should spin up then engage ball path
    * Verify speeds in dashboard

[x] Test shoot mode
    * Holding X will run in shoot mode
    * Should spin up then engage ball path
    * Verify speeds in dashboard

* Find your shooting distance
    * Find it
    * Measure to center of hub

[x] Test vision
    * Enable it  
    * Right stick button resets pose
    * Look for error on dashboard
    * Check pose in AdvantageScope

* Test driving to shooting pose
    * Make sure you are set to blue alliance
    * Make sure you've set the shot distance in preferences
    * Reset pose from vision (confirm in AdvantageScope)
    * Left bumper = orient only (must hold)
    * Right bumper = orient and shoot (must hold)

* Run auto
    * Pick one & run it
    * Watch for commands in console
    * Watch pose in AdvantageScope
 
* Implement auto
    * Find intake poses in PP
    * Unload = shoot for a short period of time
    * DriveToXxx = swerve to pose command

* Reset commands to orbit around hub and test
    * Make sure you are set to blue alliance
    * Reset pose from vision (confirm in AdvantageScope)
    * Drive to shooting position
    * Orbit around hub

* Drive over the hump a bunch of times and see how badly it throws off the
vision vs odometry. Same thing if we "surf" over a bunch of balls.  (Theory:
if we do this one too many times, we may need to manually reset the odometry
during a match.)

* Drive over the hump a bunch of times and see how badly it throws off the
vision vs odometry. (Theory: if we do this one too many times, we may need
to manually reset the odometry during a match.)

# OPEN QUESTIONS

* Is there some kind of manual targeting mode, as a fallback in case the 
odometry doesn't work?


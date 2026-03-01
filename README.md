# TEST PLAN

* Tune the new shooter wheel
    * Holding B will run shooter wheel only
    * Speeds in ShooterSubsystem
    * Look for "at speed" on the dashboard

* Test swerve modes
    * Left trigger = sniper
    * Right trigger = turbo
    * Right bumper = orbit starting position
        * Will rotate to face position
        * Right stick = spin around
        * Left stick = forward/back
        * Left stick button zeros pose

* Test intake mode
    * Holding A will run in take mode
    * Should spin up then engage ball path
    * Verify speeds in dashboard

* Find your shooting distance
    * Find it
    * Measure to center of hub

* Test vision
    * Enable it  
    * Right stick button resets pose
    * Look for error on dashboard
    
* Reset commands to orbit around hub and test
    * Make sure you are set to blue alliance
    * Reset pose from vision (confirm in AdvantageScope)
    * Drive to shooting position
    * Orbit around hub

* Run auto
    * Pick one & run it
    * Watch for commands in console
    * Watch pose in AdvantageScope

* Implement auto
    * Find intake poses in PP
    * Unload = shoot for a short period of time
    * DriveToXxx = swerve to pose command

* Drive over the hump a bunch of times and see how badly it throws off the
vision vs odometry. Same thing if we "surf" over a bunch of balls.  (Theory:
if we do this one too many times, we may need to manually reset the odometry
during a match.)

* Drive over the hump a bunch of times and see how badly it throws off the
vision vs odometry. (Theory: if we do this one too many times, we may need
to manually reset the odometry during a match.)

* Test an autonomous routine.

# OPEN QUESTIONS

* Is there some kind of manual targeting mode, as a fallback in case the 
odometry doesn't work?


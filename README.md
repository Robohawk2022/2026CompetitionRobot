# TEST PLAN

* Use the `LauncherTestbot` to tune the launch motors. Focus on being able
to hit and maintain an arbitrary target speed; we'll tweak the speeds once
we start game testing.

* Test swerve teleop (driver relative on both alliances, turbo and sniper
mode), and tune the maximum speeds for translate & rotate to comfortable
levels.

* Test jiggling and make sure it works.

* Test vision - make sure we can recognize Limelight tags and reset our
pose based on vision. (We probably only need the tags on the hub for this.)
Ensure it works for both alliances.

* Drive over the hump a bunch of times and see how badly it throws off the
vision vs odometry. Same thing if we "surf" over a bunch of balls.  (Theory:
if we do this one too many times, we may need to manually reset the odometry
during a match.)

* Drive over the hump a bunch of times and see how badly it throws off the
vision vs odometry. (Theory: if we do this one too many times, we may need
to manually reset the odometry during a match.)

* Test driving to a shooting position. Pay particular attention to accuracy;
we may need to alter the feedback calculations or other tuning parameters in
SwerveToPoseCommand.

* Test intaking balls. Tune speeds for the various motors, and consider how 
ejection may need to work.

* Test shooting. Pick one shooting distance and get reliable from there. 
Then move the robot to a different angle at the same distance, to see if 
we're still accurate.

* Test an autonomous routine.

# OPEN QUESTIONS

* Is there some kind of manual targeting mode, as a fallback in case the 
odometry doesn't work?


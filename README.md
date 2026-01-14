# TODO

Implement the initial ShooterSubsystem using the prototype
of the three-wheel shooter. Use the corresponding Testbot
to test it out, tune the PIDs and find some good shooting
formulas.

Use the Limelight testbot to explore what happens when the
Limelight has multiple AprilTags in view. Can we tell it
which one to use? What numbers might be the best for zeroing
in on a tag for targeting.

Wire up the Grapple Robotics laser rangefinder and experiment
with the API. (Will require the rangefinder to be wired to
the testbench, and the libgrapplefrc vendor dependency.)

Generate a swerve drive implementation, either with YAGSL
or CTRE project generator. (Will require some guesses as to
the chassis dimensions, and won't be fully testable until
we get the "skateboard" from the build team.)

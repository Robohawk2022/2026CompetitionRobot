# TODO

Implement the initial ShooterSubsystem using the prototype
of the three-wheel shooter.

Generate a swerve drive implementation, either with YAGSL
or CTRE project generator. (Will require some guesses as to
the chassis dimensions, and won't be fully testable until
we get the "skateboard" from the build team.)

Work with the Limelight to see what it does with multiple
tags in view. Devise a plan for what targeting "modes" we
will need, and create the relevant pipelines. (Will just
require the RoboRIO hooked up to the Limelight, since all
the relevant output is in NetworkTables.)

Wire up the Grapple Robotics laser rangefinder and experiment
with the API. (Will require the rangefinder to be wired to
the testbench, and the libgrapplefrc vendor dependency.)

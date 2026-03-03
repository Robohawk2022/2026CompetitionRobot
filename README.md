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

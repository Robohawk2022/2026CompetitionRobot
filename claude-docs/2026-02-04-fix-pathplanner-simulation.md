# Session: Fix PathPlanner Simulation

**Date**: 2026-02-04
**Duration**: ~30 minutes (continuation of previous session)

## Summary

Fixed two issues preventing PathPlanner autonomous from working in simulation: the teleop command was overwriting PathPlanner's output, and the simulation odometry integration wasn't working.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/subsystems/auto/AutonomousSubsystem.java`
  - Added `swerve` as a requirement to the decorated auto command so SwerveTeleopCommand (default command) gets interrupted during autonomous

- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
  - Fixed `simulationPeriodic()` condition: changed from `.equals(Util.ZERO_SPEED)` to explicit value comparison (`Math.abs() > 0.001`) since ChassisSpeeds equality wasn't working as expected

## Technical Details

### Issue 1: Teleop Command Interfering with Auto

**Problem**: PathPlannerAuto uses a callback-based output (`BiConsumer<ChassisSpeeds, DriveFeedforwards>`) instead of requiring the swerve subsystem. This meant the default SwerveTeleopCommand continued running during autonomous, overwriting PathPlanner's speed commands.

**Symptoms**:
- Mode showed "teleop" during autonomous
- PP_X/PP_Y showed non-zero speeds but SwerveSubsystem/SpeedX/Y showed 0
- Both SequentialCommandGroup and SwerveTeleopCommand appeared in active commands

**Fix**: Modified `decorateAutoCommand()` to wrap the auto command with a `Commands.runOnce()` that requires the swerve subsystem:
```java
Command requireSwerve = Commands.runOnce(() -> {
    String alliance = Util.isRedAlliance() ? "RED" : "BLUE";
    Util.log("[auto] starting auto as %s alliance", alliance);
}, swerve);  // <-- swerve requirement added here
```

### Issue 2: Simulation Odometry Not Updating

**Problem**: The `simulationPeriodic()` method wasn't integrating speeds because `latestSpeed.equals(Util.ZERO_SPEED)` always returned true (likely due to object reference comparison rather than value comparison).

**Symptoms**:
- PP_X showed speeds being commanded
- SpeedX showed speeds being captured
- But PoseX/PoseY never changed from 0

**Fix**: Changed the zero-check to explicit value comparison:
```java
boolean hasMovement = latestSpeed != null &&
    (Math.abs(latestSpeed.vxMetersPerSecond) > 0.001 ||
     Math.abs(latestSpeed.vyMetersPerSecond) > 0.001 ||
     Math.abs(latestSpeed.omegaRadiansPerSecond) > 0.001);
```

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested - robot moves during autonomous
  - Autonomous command runs (Running? = True)
  - Mode shows "path-planner" during auto
  - PP_X/PP_Y show PathPlanner speeds
  - SpeedX/SpeedY show captured speeds
  - PoseX/PoseY/PoseOmega update during path execution
  - Robot moved from (0, 0) to approximately (10 ft, 21.5 ft)

## Known Issues / TODO

- [ ] The MCP `get_robot_pose` tool reads from a different field than SmartDashboard - should use `/SmartDashboard/SwerveSubsystem/PoseX` etc.
- [ ] ZigZag path waypoints may need adjustment - final position didn't match expected endpoint
- [ ] PathPlanner telemetry logging (target pose, active path) should be verified in PathPlanner GUI

## Notes for Next Session

The simulation now works for testing PathPlanner autonomous routines:
1. Start simulation: `./gradlew simulateJava`
2. Enable autonomous via simulation GUI or MCP tools
3. Monitor pose via SmartDashboard keys: `SwerveSubsystem/PoseX`, `SwerveSubsystem/PoseY`, `SwerveSubsystem/PoseOmega`
4. Or via simple keys: `Pose X (ft)`, `Pose Y (ft)`, `Pose Heading (deg)`

The robot position now updates correctly during autonomous path following in simulation.

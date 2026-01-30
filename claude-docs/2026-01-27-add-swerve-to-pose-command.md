# Session: Add SwerveToPoseCommand

**Date**: 2026-01-27

## Summary

Created `SwerveToPoseCommand` for autonomous driving to a target pose along a straight line. Uses two independent trapezoidal motion profiles - one for translation, one for rotation.

## Changes Made

### Files Created
- `src/main/java/frc/robot/commands/swerve/SwerveToPoseCommand.java` - Command for driving to a target pose

### Files Modified
- `src/main/java/frc/robot/Config.java`
  - Added translation parameters to `SwerveAuto` interface (velocity, acceleration, tolerance)

- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
  - Added `toPoseCommand(Pose2d)` factory method

## SwerveToPoseCommand

Drives the swerve to a target pose along a straight line using two independent trapezoid profiles.

### Usage
```java
// From SwerveSubsystem
Pose2d target = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90));
Command driveToTarget = swerve.toPoseCommand(target);

// Or directly
Command driveToTarget = new SwerveToPoseCommand(swerve, targetPose);
```

### Config.SwerveAuto Parameters

**Translation:**
- `SwerveAuto/MaxTranslationFPS` (default: 8.0) - Maximum translation velocity in ft/s
- `SwerveAuto/MaxTranslationFPSS` (default: 12.0) - Maximum translation acceleration in ft/s²
- `SwerveAuto/PositionToleranceFeet` (default: 0.1) - Position tolerance in feet

**Rotation:**
- `SwerveAuto/MaxRotationDPS` (default: 180) - Maximum rotation velocity in deg/s
- `SwerveAuto/MaxRotationDPSS` (default: 360) - Maximum rotation acceleration in deg/s²
- `SwerveAuto/HeadingToleranceDeg` (default: 2) - Heading tolerance in degrees

### Behavior
1. **initialize()**:
   - Calculates straight-line path from current pose to target
   - Computes unit direction vector for translation
   - Calculates shortest rotation path (-180 to 180 wrapping)
   - Configures both trapezoid profiles

2. **execute()**:
   - Samples both profiles at current time
   - Computes field-relative velocity (speed × direction unit vector)
   - Converts to robot-relative speeds
   - Drives the swerve

3. **isFinished()**:
   - Both profiles must be complete
   - Position must be within tolerance of target
   - Heading must be within tolerance of target

4. **end()**:
   - Stops the drive
   - Logs completion status

### Dashboard (verbose logging)
- `Running?` - is the command scheduled
- `TotalDistance` - total shotType to travel (feet)
- `CurrentDistance` - current shotType from target (feet)
- `ProfileDistance` - current profile position (feet)
- `ProfileVelocity` - current profile velocity (ft/s)
- `TargetHeading` - target heading (degrees)
- `CurrentHeading` - current gyro heading (degrees)
- `ProfileHeading` - current profile heading (degrees)
- `ProfileOmega` - current profile angular velocity (deg/s)

## Architecture Notes

The command uses field-relative velocities internally:
1. Translation velocity is computed as `speed × directionUnitVector`
2. This gives field-relative vx/vy
3. Converted to robot-relative using current heading before driving

This approach ensures the robot follows a straight line regardless of its heading during the motion.

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not tested
- [ ] Real hardware - not tested

## Known Issues / TODO

- No feedback loop - uses pure feedforward from profiles
- Both profiles run independently - rotation may complete before/after translation
- Consider adding option for "rotate first, then translate" or "translate first, then rotate" modes

## Notes for Next Session

The command assumes the robot can translate and rotate simultaneously. For situations where this causes issues (e.g., wheel scrub), consider:
1. Sequencing the commands: `swerve.toHeadingCommand(heading).andThen(swerve.toPoseCommand(pose))`
2. Adding a "rotation priority" mode that delays translation until heading is close

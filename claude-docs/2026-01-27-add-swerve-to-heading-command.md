# Session: Add SwerveToHeadingCommand

**Date**: 2026-01-27

## Summary

Created `SwerveToHeadingCommand` for autonomous heading control using trapezoidal motion profiling. Added `Config.SwerveAuto` interface for heading control parameters.

## Changes Made

### Files Created
- `src/main/java/frc/robot/commands/swerve/SwerveToHeadingCommand.java` - Command for rotating to a target heading

### Files Modified
- `src/main/java/frc/robot/Config.java`
  - Added `SwerveAuto` interface with heading control parameters

- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
  - Added `toHeadingCommand(Rotation2d)` factory method

## SwerveToHeadingCommand

Rotates the swerve drive to face a target heading using trapezoidal motion profiling.

### Usage
```java
// From SwerveSubsystem
Command turnTo90 = swerve.toHeadingCommand(Rotation2d.fromDegrees(90));

// Or directly
Command turnToTarget = new SwerveToHeadingCommand(swerve, targetHeading);
```

### Config.SwerveAuto Parameters
- `SwerveAuto/MaxRotationDPS` (default: 180) - Maximum rotation velocity in deg/s
- `SwerveAuto/MaxRotationDPSS` (default: 360) - Maximum rotation acceleration in deg/s²
- `SwerveAuto/HeadingToleranceDeg` (default: 2) - Tolerance for completion

### Behavior
1. On initialize: calculates shortest path to target (handles -180 to 180 wrapping)
2. On execute: samples trapezoid profile and applies rotation speed
3. Finishes when profile completes AND heading is within tolerance
4. On end: stops the drive

### Dashboard (verbose logging)
- `Running?` - is the command scheduled
- `StartHeading` - heading at command start
- `TargetHeading` - target heading (adjusted for shortest path)
- `CurrentHeading` - current gyro heading
- `ProfilePosition` - current profile position in degrees
- `ProfileVelocity` - current profile velocity in deg/s

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not tested
- [ ] Real hardware - not tested

## Known Issues / TODO

- None identified

## Notes for Next Session

The command uses pure feedforward from the trapezoid profile (no feedback loop). If heading accuracy is insufficient, consider adding a PD feedback term using the error between profile position and actual heading.

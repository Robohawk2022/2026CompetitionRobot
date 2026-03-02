# Session: Fix SwerveToPoseCommand Signed Distance Error

**Date**: 2026-03-02

## Summary

Fixed a bug in `SwerveToPoseCommand` where the translation distance error was computed as an unsigned (always-positive) value, preventing the command from correcting overshoot.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/commands/swerve/SwerveToPoseCommand.java` — fixed distance error calculation in `execute()`

## Bug Description

The command models translation as a 1D journey from 0 (start) to `distanceGoalFeet` (target). However, the "current distance" was computed using `Util.feetBetween(swerve.getPose(), targetPose)`, which calls `.getNorm()` and is always non-negative.

This meant `distanceErrorFeet` was always positive, so `distanceFeedbackFps = kP * distanceErrorFeet` always commanded forward velocity. Stopping 4 feet short and overshooting by 4 feet both produced the same positive feedback — the robot could not correct overshoot.

## Fix

Replaced the unsigned `feetBetween` call with a signed projection onto the path direction:

```java
// before (unsigned):
distanceCurrentFeet = Util.feetBetween(swerve.getPose(), targetPose);
distanceErrorFeet = distanceCurrentFeet;

// after (signed 1D progress from start):
Translation2d traveled = swerve.getPose().getTranslation().minus(startPose.getTranslation());
distanceCurrentFeet = Units.metersToFeet(traveled.dot(directionUnit));
distanceErrorFeet = distanceDesiredFeet - distanceCurrentFeet;
```

`distanceCurrentFeet` now represents how far the robot has traveled along the path from start (0 → distanceGoalFeet normally, >distanceGoalFeet if overshot). `distanceErrorFeet` is the signed error vs the profile's desired position: positive = behind, negative = overshot/ahead.

This also changes the dashboard `DistanceCurrent` semantics from "distance remaining to target" to "distance traveled from start," which is more natural (matches the profile's 0 → goal range).

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation / hardware - not tested yet

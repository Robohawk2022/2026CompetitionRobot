# Session: Add Swerve Module Rotation Logging

**Date**: 2026-01-19

## Summary

Added motor rotation logging to Shuffleboard for debugging swerve modules. The logging shows drive and turn motor rotations along with their CAN IDs, gated by the existing `Logging/Swerve?` config preference.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`
  - Added `driveId`, `turnId`, `encoderId` fields to store CAN IDs
  - Added verbose logging in SmartDashboard.putData block gated by `Config.Logging.swerveLogging`
  - Added import for `frc.robot.Config`

## Dashboard Output

When `Logging/Swerve?` is enabled (and `Logging/VerboseEnabled?` is true):

```
Swerve/Module-FL/
  Velocity          (existing)
  Angle             (existing)
  DesiredVelocity   (existing)
  DesiredAngle      (existing)
  DriveRotations    (NEW - drive motor rotations)
  TurnRotations     (NEW - turn encoder rotations)
  DriveCANID        (NEW - integer)
  TurnCANID         (NEW - integer)
  EncoderCANID      (NEW - integer)
```

## Config Usage

The existing preference `Logging/Swerve?` (default: false) controls this verbose output. No new config entries needed.

To enable in Shuffleboard/NetworkTables:
- Set `Logging/VerboseEnabled?` = true (master switch)
- Set `Logging/Swerve?` = true

## CAN ID Reference

From Config.java:
- FL: Drive=41, Turn=42, Encoder=43
- FR: Drive=21, Turn=22, Encoder=23
- BL: Drive=11, Turn=12, Encoder=13
- BR: Drive=31, Turn=32, Encoder=33

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

The rotation values are raw motor/encoder rotations (not converted to wheel rotations or radians). This is useful for debugging encoder issues and verifying motor movement.

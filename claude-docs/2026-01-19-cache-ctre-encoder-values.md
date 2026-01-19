# Session: Cache CTRE Encoder Values in Periodic

**Date**: 2026-01-19

## Summary

Implemented caching for CTRE Phoenix 6 status signals to reduce CAN bus traffic. All encoder and gyro values are now batch-refreshed once per cycle using `BaseStatusSignal.refreshAll()` instead of triggering individual CAN reads on each getter call.

## Changes Made

### Files Modified

#### `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`
- Added `driveVelocitySignal` StatusSignal field
- Added cached value fields: `cachedDrivePositionMeters`, `cachedDriveVelocityMps`, `cachedTurnPositionRad`
- Added `refreshSignals()` method that updates cached values from status signals
- Updated `getState()` and `getPosition()` to return cached values instead of triggering CAN reads
- Updated `getSignals()` to include the velocity signal (now returns 3 signals instead of 2)

#### `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java`
- Added `cachedHeading` field
- Updated `allSignals` array collection to use module's `getSignals()` method (now includes velocity)
- Implemented `refreshSignals()` override that:
  - Calls `BaseStatusSignal.refreshAll()` to batch-refresh all 13 signals in one CAN transaction
  - Calls `refreshSignals()` on each module to update cached values
  - Updates cached heading from yaw signal
- Updated `getHeading()` to return cached heading value
- Added initial `refreshSignals()` call in constructor to populate cached values

#### `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java`
- Added `refreshSignals()` default method to interface (no-op for simulation)

#### `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
- Added `hardware.refreshSignals()` call at the start of `periodic()`

## Technical Details

### Before (Problem)
Every call to `getState()` or `getPosition()` in the periodic loop triggered direct CAN reads:
- `driveMotor.getVelocity().getValueAsDouble()` - CAN read
- `driveMotor.getPosition().getValueAsDouble()` - CAN read
- `turnEncoder.getAbsolutePosition().getValueAsDouble()` - CAN read
- `pigeon.getRotation2d()` - CAN read

This caused ~16+ CAN reads per 50Hz cycle when called from `SwerveSubsystem.periodic()`.

### After (Solution)
All 13 status signals (3 per module + 1 gyro) are batch-refreshed in a single CAN transaction using `BaseStatusSignal.refreshAll()`. Subsequent calls to getters return cached values without any CAN traffic.

Signal count per module: drive position + drive velocity + turn position = 3
Total signals: 4 modules * 3 signals + 1 gyro yaw = 13 signals

## Testing Done

- [x] `./gradlew build` - passed

## Known Issues / TODO

- [ ] Test on real robot to verify encoder values update correctly
- [ ] Monitor CAN bus utilization to confirm reduction
- [ ] The high-frequency odometry thread (OdometryThread) is still commented out - may need updates if re-enabled

## Notes for Next Session

The simulation (`SwerveSim`) doesn't need signal caching since there's no CAN bus, so the default no-op implementation in `SwerveHardware` is appropriate. The cached values are initialized in the constructor via `refreshSignals()` to ensure they have valid values before the first periodic call.

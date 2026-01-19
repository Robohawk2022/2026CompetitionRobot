# Session: Remove High-Frequency Odometry Thread

**Date**: 2026-01-19

## Summary

Removed the disabled high-frequency `OdometryThread` implementation and simplified to periodic-only (50Hz) odometry updates. This removes dead/disabled code and reduces complexity.

## Background

The high-frequency odometry thread was designed to run at 250Hz using CTRE synchronized signal reads, but it was disabled in a previous session because it overwhelmed the CAN bus. Since it was not being used, this session removes it entirely to simplify the codebase.

## Changes Made

### Files Deleted
- `src/main/java/frc/robot/subsystems/swerve/OdometryThread.java` - HF odometry thread class
- `src/main/java/frc/robot/util/TimestampedPose.java` - Record used only by OdometryThread

### Files Modified

| File | Changes |
|------|---------|
| `SwerveSubsystem.java` | Removed `odometryThread` field, constructor initialization, and all HF odometry methods (`getOdometryThread()`, `isHighFrequencyOdometryEnabled()`, `getOdometryPoseAtTime()`). Simplified `periodic()` and `resetPose()` to use standard 50Hz odometry only. |
| `SwerveHardware.java` | Removed `getAllSignals()` and `supportsHighFrequencyOdometry()` default methods. Removed `BaseStatusSignal` import. |
| `SwerveHardwareCTRE.java` | Removed `getAllSignals()` override method (kept `allSignals` array since it's still used by `refreshSignals()`). |
| `SwerveModule.java` | Removed `getDrivePositionSignal()` and `getTurnPositionSignal()` getter methods (kept `getSignals()` since it's used for signal batching). |
| `OdometryDiagnostics.java` | Removed `OdometryThread` parameter from `update()` method. Removed odometry frequency tracking (`updateFrequencyEstimate()`, `getOdometryFrequency()`, related fields). |
| `Config.java` | Removed `HF_FREQUENCY`, `POSE_HISTORY_SIZE`, and `enableHF` from `Odometry` interface. |

### Files Kept
- `CircularBuffer.java` - Still used by `OdometryDiagnostics` for drift history tracking

## Config Changes

- Removed `Odometry/EnableHF?` preference
- Removed `HF_FREQUENCY` constant (was 100.0 Hz)
- Removed `POSE_HISTORY_SIZE` constant (was 50)

## Testing Done

- [x] `./gradlew build` - passed
- [x] No remaining references to removed code in src/

## Architecture Notes

The swerve subsystem now uses standard 50Hz periodic odometry updates:
1. `periodic()` calls `hardware.refreshSignals()` to batch CAN reads
2. Standard `SwerveDriveOdometry.update()` and `SwerveDrivePoseEstimator.update()` are called
3. Vision updates are applied via `LimelightEstimator.updateEstimate()`

This is sufficient for most FRC applications. Many championship teams use 50Hz odometry successfully. The ~1-2cm extra drift at typical speeds compared to 250Hz is negligible for most use cases.

## Future Considerations

If higher-frequency odometry is needed in the future, consider migrating to CTRE's `SwerveDrivetrain` class which handles high-frequency odometry internally with proper CAN bus management. This would be a larger refactor that deserves its own planning session.

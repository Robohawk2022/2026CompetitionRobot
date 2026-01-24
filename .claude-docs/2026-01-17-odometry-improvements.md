# Session: Odometry Improvements - High-Frequency Odometry & Diagnostics

**Date**: 2026-01-17
**Duration**: ~30 minutes

## Summary

Implemented high-frequency odometry (250Hz) using CTRE synchronized reads with latency compensation, plus comprehensive dashboard diagnostics for tracking drift, vision rejection stats, and odometry health.

## Changes Made

### Files Created

| File | Description |
|------|-------------|
| `src/main/java/frc/robot/util/TimestampedPose.java` | Record for pose with timestamp, used for latency compensation |
| `src/main/java/frc/robot/util/CircularBuffer.java` | Generic circular buffer for pose history and rolling statistics |
| `src/main/java/frc/robot/subsystems/swerve/OdometryThread.java` | High-frequency (250Hz) odometry thread using CTRE `BaseStatusSignal.waitForAll()` |
| `src/main/java/frc/robot/subsystems/swerve/OdometryDiagnostics.java` | Tracks and publishes odometry health metrics to dashboard |

### Files Modified

| File | Changes |
|------|---------|
| `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java` | Added StatusSignal fields with 250Hz update frequency, getter methods |
| `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` | Added `getAllSignals()` and `supportsHighFrequencyOdometry()` interface methods |
| `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` | Implemented `getAllSignals()` with 9 signals (8 module + gyro yaw) |
| `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` | Integrated OdometryThread and OdometryDiagnostics, added latency compensation support |
| `src/main/java/frc/robot/Config.java` | Added `Odometry` config interface |

## Config Changes

Added `Config.Odometry` interface with:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `HF_FREQUENCY` | 250.0 Hz | High-frequency odometry update rate |
| `POSE_HISTORY_SIZE` | 50 | Poses to keep for latency compensation (~200ms at 250Hz) |
| `Odometry/EnableHF?` | true | Enable/disable high-frequency odometry |
| `Odometry/Diagnostics?` | true | Enable/disable diagnostics publishing |
| `Odometry/DriftWarning_m` | 0.5 | Drift threshold for console warning |

## Dashboard Keys Published

| Key | Description |
|-----|-------------|
| `Odom/DriftCurrent_m` | Current drift between odometry and fused pose |
| `Odom/DriftMax_m` | Maximum drift this session |
| `Odom/DriftAvg_m` | Rolling average drift |
| `Odom/VisionAcceptRate_%` | Percentage of accepted vision estimates |
| `Odom/VisionLatency_ms` | Latest vision measurement latency |
| `Odom/VisionAccepted` | Count of accepted vision estimates |
| `Odom/VisionRejected` | Count of rejected vision estimates |
| `Odom/OdomFrequency_Hz` | Actual odometry update frequency |
| `Odom/Reject/*` | Rejection counts by reason (NoTags, Ambiguity, TooFar, etc.) |

## Architecture

### High-Frequency Odometry Flow

```
OdometryThread (250Hz via Notifier)
    │
    ├── BaseStatusSignal.waitForAll() - synchronized read of 9 signals
    │       ├── FL: drive position, turn position
    │       ├── FR: drive position, turn position
    │       ├── BL: drive position, turn position
    │       ├── BR: drive position, turn position
    │       └── Pigeon2 yaw
    │
    ├── SwerveDriveOdometry.update() - calculates pose
    │
    └── CircularBuffer<TimestampedPose> - stores for latency compensation
```

### Latency Compensation

The `OdometryThread` stores a history of timestamped poses in a circular buffer. When vision measurements arrive with a timestamp, `getPoseAtTime(timestamp)` returns an interpolated pose from the history, allowing accurate fusion even with vision latency.

### Simulation Support

The system gracefully degrades when high-frequency odometry isn't supported:
- `SwerveSim` returns empty signal array from `getAllSignals()`
- `SwerveSubsystem` detects this and falls back to standard 50Hz odometry
- Diagnostics still track drift and vision stats at 50Hz

## Methods Added

### SwerveModule

| Method | Description |
|--------|-------------|
| `getDrivePositionSignal()` | Returns drive position StatusSignal for synchronized reads |
| `getTurnPositionSignal()` | Returns turn position StatusSignal for synchronized reads |
| `getSignals()` | Returns array of both signals for this module |

### SwerveHardware

| Method | Description |
|--------|-------------|
| `getAllSignals()` | Returns all signals for synchronized odometry reads |
| `supportsHighFrequencyOdometry()` | Returns true if HF odometry is supported |

### SwerveSubsystem

| Method | Description |
|--------|-------------|
| `getDiagnostics()` | Returns the OdometryDiagnostics instance |
| `getOdometryThread()` | Returns the OdometryThread (may be null) |
| `isHighFrequencyOdometryEnabled()` | Returns true if HF odometry is running |
| `getOdometryPoseAtTime(timestamp)` | Returns interpolated pose for latency compensation |

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not run (needs SwerveTestbot)
- [ ] Real hardware tested - not tested yet

## Known Issues / TODO

- [ ] Test in simulation with `./gradlew simulateJava -Probot=SwerveTestbot`
- [ ] Verify dashboard shows correct diagnostics in AdvantageScope
- [ ] Tune drift warning threshold based on actual robot performance
- [ ] Consider adding rate limiting to drift warnings (currently logs every cycle)

## Notes for Next Session

1. The OdometryThread uses a `Notifier` which is a separate thread. The `ReadWriteLock` ensures thread-safe access to the pose.

2. The signal array order in `SwerveHardwareCTRE.getAllSignals()` must match the extraction order in `OdometryThread.extractModulePosition()`:
   - Index 0-1: FL drive, turn
   - Index 2-3: FR drive, turn
   - Index 4-5: BL drive, turn
   - Index 6-7: BR drive, turn
   - Index 8: Pigeon yaw

3. Vision latency compensation can be added by modifying `LimelightEstimator.updateEstimate()` to use `getOdometryPoseAtTime()` when determining the robot pose at the time of the vision measurement.

4. The diagnostics categorization for rejection reasons uses string matching on the rejection message. If new rejection reasons are added to `LimelightEstimator`, consider updating the `categorizeRejection()` method.

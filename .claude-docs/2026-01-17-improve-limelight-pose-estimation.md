# Session: Improve Limelight AprilTag Pose Estimation

**Date**: 2026-01-17

## Summary

Improved the Limelight vision pose estimation for better robot localization. Integrated vision into the swerve drive pose estimator with dynamic standard deviations, field boundary validation, and pose jump detection.

## Changes Made

### Files Created

| File | Description |
|------|-------------|
| `src/main/java/frc/robot/subsystems/vision/VisionEstimateResult.java` | Result class for vision processing containing estimate, std devs, acceptance status, and rejection reason |

### Files Modified

| File | Changes |
|------|---------|
| `src/main/java/frc/robot/subsystems/swerve/SwerveHardware.java` | Added `getYawRateDps()` method to interface |
| `src/main/java/frc/robot/subsystems/swerve/SwerveHardwareCTRE.java` | Implemented `getYawRateDps()` using Pigeon2 angular velocity |
| `src/main/java/frc/robot/subsystems/swerve/SwerveSim.java` | Implemented `getYawRateDps()` from chassis speeds omega |
| `src/main/java/frc/robot/Config.java` | Expanded `Limelight` interface with tunable vision parameters |
| `src/main/java/frc/robot/subsystems/vision/LimelightEstimator.java` | Complete refactor with new features (see below) |
| `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` | Integrated vision into `periodic()`, added getters |

## Config Changes

Expanded `Config.Limelight` interface with:

**Enable/Strategy:**
- `Vision/Enabled?` = true - Master enable/disable
- `Vision/Strategy` = 1.0 - 0=Classic, 1=MegaTag2, 2=Both

**Standard Deviation Tuning:**
- `Vision/BaseStdDev/XY_1Tag` = 0.9 - Trust level for single tag
- `Vision/BaseStdDev/XY_MultiTag` = 0.5 - Trust level for multi-tag
- `Vision/DistScaleFactor` = 3.0 - Distance where std dev doubles

**Filtering Thresholds:**
- `Vision/MaxAmbiguity` = 0.7 - Single-tag ambiguity threshold
- `Vision/MaxDistToCamera` = 4.0 - Max trusted distance (meters)
- `Vision/MaxYawRateDPS` = 720.0 - Max rotation for MegaTag2
- `Vision/MinTagArea` = 0.1 - Min tag size for MegaTag2
- `Vision/MaxPoseJump` = 2.0 - Max allowed pose change (meters)

## LimelightEstimator Improvements

1. **Fixed MegaTag2 bug** - Now uses `getBotPoseEstimate_wpiBlue_MegaTag2()` instead of `getBotPoseEstimate_wpiBlue()`

2. **Dynamic standard deviation** - Formula: `stdDev = baseStdDev * (1 + distance/scaleFactor) / sqrt(tagCount)`
   - More tags = lower std dev (more trust)
   - Farther distance = higher std dev (less trust)

3. **Field boundary validation** - Rejects poses outside 16.54m x 8.21m (with margin)

4. **Pose jump detection** - Rejects if >2m from current estimate

5. **Configurable thresholds** - All filtering parameters read from Config

6. **Main entry point** - New `updateEstimate()` method handles strategy selection

7. **Enhanced telemetry:**
   - `Vision/Valid` - Is estimate accepted?
   - `Vision/Status` - Current status or rejection reason
   - `Vision/TagCount` - Number of tags seen
   - `Vision/AvgTagDist` - Average distance to tags
   - `Vision/StdDevXY` - Calculated standard deviation
   - `Vision/IsMegaTag2` - Which strategy produced result
   - `Vision/PoseX/Y/Rot` - Vision-reported pose
   - `Vision/RejectionReason` - Why estimate was rejected

## SwerveSubsystem Integration

- Vision update now called in `periodic()` with:
  - Pose estimator
  - Current yaw from gyro
  - Yaw rate from new `getYawRateDps()` method
  - Current pose for jump detection

- `resetPose()` now calls `LimelightEstimator.resetLastPose()` to reset jump detection baseline

- Added getters:
  - `isVisionValid()` - Quick check if latest vision was accepted
  - `getLatestVisionResult()` - Full result for debugging

## Testing Done

- [x] `./gradlew build` - Passed
- [ ] Simulation tested - Not yet tested
- [ ] Real hardware - Not tested

## Known Issues / TODO

- [ ] Test in simulation with `./gradlew simulateJava -Probot=SwerveTestbot`
- [ ] Verify dashboard telemetry displays correctly
- [ ] Tune std dev parameters on real field
- [ ] Adjust field boundary constants if using different field year
- [ ] Consider adding latency compensation (estimate timestamp vs current time)

## Notes for Next Session

1. To test vision integration in simulation:
   ```bash
   ./gradlew simulateJava -Probot=SwerveTestbot
   ```

2. Dashboard values to monitor:
   - `Vision/Valid` should be true when tags visible
   - `Vision/TagCount` shows detected tags
   - `Vision/StdDevXY` varies with distance and tag count
   - `Vision/RejectionReason` shows why rejected

3. Compare poses in AdvantageScope:
   - `PoseEstimates/Odometry` - Wheel odometry only
   - `PoseEstimates/Estimated` - Fused with vision
   - `PoseEstimates/Vision` - Raw vision pose

4. The std dev formula can be tuned:
   - Increase `baseStdDev` values to trust vision less
   - Decrease `distScaleFactor` to penalize distance more
   - The `/sqrt(tagCount)` factor gives multi-tag ~1.4x better trust than single

5. Legacy methods (`updateEstimateClassic`, `updateEstimateMegaTag2`) kept for backwards compatibility but marked `@Deprecated`

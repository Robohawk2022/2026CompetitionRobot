# Session: Fix CTRE Swerve Odometry Issues

**Date**: 2026-02-01
**Duration**: ~5 minutes

## Summary

Fixed critical odometry issue where vision rotation measurements were completely ignored due to theta standard deviations being set to `9999999`. Also increased max pose jump threshold.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Fixed vision theta standard deviations and increased max pose jump

## Config Changes

### CRITICAL FIX: Vision Theta Standard Deviations

**Before (broken):**
```java
Vector<N3> lowConfidence    = VecBuilder.fill(2.0, 2.0, 9999999);
Vector<N3> mediumConfidence = VecBuilder.fill(0.9, 0.9, 9999999);
Vector<N3> highConfidence   = VecBuilder.fill(0.5, 0.5, 9999999);
```

**After (fixed):**
```java
Vector<N3> lowConfidence    = VecBuilder.fill(2.0, 2.0, 0.5);   // ~28 deg theta
Vector<N3> mediumConfidence = VecBuilder.fill(0.9, 0.9, 0.3);   // ~17 deg theta
Vector<N3> highConfidence   = VecBuilder.fill(0.5, 0.5, 0.1);   // ~6 deg theta
```

**Impact**: The Kalman filter was configured to completely ignore vision rotation measurements. This caused:
- Robot heading drifts over time (gyro drift accumulates with no correction)
- Long autonomous routines become increasingly inaccurate
- Vision could only correct X/Y position, never heading

### MEDIUM FIX: Max Pose Jump Threshold

- Changed `Limelight/MaxPoseJumpFeet` default from `1.0` to `2.0`
- Previous 1 foot threshold was too strict; once odometry drifted more than 1 foot from reality, vision corrections were rejected entirely

## Testing Done

- [x] `./gradlew build` - passed

## Verification Steps (to be done on real robot)

1. Run simulation: `./gradlew simulateJava`
2. Test on real robot:
   - Rotate robot in place for 30 seconds
   - Check if vision corrects any gyro drift
   - Monitor `VisionPoseCount` on dashboard to confirm vision measurements are being accepted

## Notes for Next Session

The theta standard deviations may need further tuning based on real-world testing:
- If heading is jittery: increase theta std devs (less vision trust)
- If heading still drifts: decrease theta std devs (more vision trust)

The values chosen are conservative starting points:
- 0.1 radians (~6 degrees) for high confidence
- 0.3 radians (~17 degrees) for medium confidence
- 0.5 radians (~28 degrees) for low confidence

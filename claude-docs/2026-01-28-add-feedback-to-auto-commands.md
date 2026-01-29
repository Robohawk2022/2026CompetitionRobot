# Session: Add Feedback to Auto Commands

**Date**: 2026-01-28

## Summary

Added proportional feedback to `SwerveToHeadingCommand` and `SwerveToPoseCommand` to fix a bug where the robot would stop moving after the trapezoid profile completed but before reaching the target tolerance. Also reorganized dashboard logging keys for better grouping in Elastic.

## Problem

The original commands used pure feedforward from the trapezoid profile. After the profile time elapsed, `Trapezoid.sample()` returns the final state with velocity = 0. If the robot wasn't within tolerance at that point, `isFinished()` would return false but `execute()` would command zero velocity - the robot would sit still forever.

## Solution

Combined feedforward (profile velocity) with proportional feedback (kP * error):
- During profile: feedforward dominates, feedback corrects drift
- After profile completes: feedforward = 0, feedback drives remaining error to zero

Chose P-only feedback (not PD) because:
1. The trapezoid profile already handles smooth motion/deceleration
2. Post-profile corrections are small adjustments
3. Simpler tuning (one parameter)

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java`
  - Added `SwerveAuto/TranslationKp` (default 1.0) for translation feedback
  - Added `SwerveAuto/RotationKp` (default 2.0) for rotation feedback
  - Reorganized with better region markers

- `src/main/java/frc/robot/commands/swerve/SwerveToHeadingCommand.java`
  - Added feedback calculation: `feedbackDps = rotationKp * headingErrorDeg`
  - Combined with feedforward: `omegaDps = velocityDesiredDeg + feedbackDps`
  - Renamed variables to match dashboard keys for clarity
  - Added dashboard logging for error and feedback values
  - Changed log prefix to `[auto-heading]`
  - Added `timer.stop()` in `end()`

- `src/main/java/frc/robot/commands/swerve/SwerveToPoseCommand.java`
  - Added translation feedback: `distanceFeedbackFps = translationKp * distanceErrorFeet`
  - Added rotation feedback: `headingFeedbackDps = rotationKp * headingErrorDeg`
  - Combined with feedforward for both axes
  - Renamed variables to match dashboard keys for clarity
  - Added dashboard logging for error and feedback values
  - Changed log prefix to `[auto-pose]`
  - Added `timer.stop()` in `end()`

## Config Changes

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SwerveAuto/TranslationKp` | 1.0 | Proportional gain for translation feedback (1/sec) |
| `SwerveAuto/RotationKp` | 2.0 | Proportional gain for rotation feedback (1/sec) |

## Dashboard Keys

Keys are named to group alphabetically in Elastic (e.g., `HeadingCurrent`, `HeadingDesired`, `HeadingError`, `HeadingGoal`).

**Naming convention:**
- `XxxCurrent` - where the robot actually is now
- `XxxDesired` - where the profile wants it to be now (moving setpoint)
- `XxxError` - difference between desired and current
- `XxxGoal` - final destination

**SwerveToHeadingCommand:**
| Key | Description |
|-----|-------------|
| HeadingStart | Starting heading (deg) |
| HeadingGoal | Final target heading (deg) |
| HeadingCurrent | Current heading (deg) |
| HeadingDesired | Profile setpoint (deg) |
| HeadingError | Goal - Current (deg) |
| VelocityDesired | Feedforward from profile (dps) |
| VelocityFeedback | kP * error correction (dps) |

**SwerveToPoseCommand:**
| Key | Description |
|-----|-------------|
| DistanceCurrent | Remaining distance to target (ft) |
| DistanceDesired | Profile position setpoint (ft) |
| DistanceError | Position error (ft) |
| DistanceGoal | Total distance to travel (ft) |
| HeadingCurrent/Desired/Error/Goal | Same pattern as above |
| VelocityDesired | Translation feedforward (fps) |
| VelocityFeedback | Translation feedback (fps) |
| OmegaDesired | Rotation feedforward (dps) |
| OmegaFeedback | Rotation feedback (dps) |

## Testing Done

- [x] `./gradlew build` - passed

## Known Issues / TODO

- [ ] Tune `TranslationKp` and `RotationKp` on real robot
- [ ] Test in simulation to verify convergence behavior

## Notes for Next Session

The feedback gains (kP) may need tuning. Start with defaults and adjust based on:
- If oscillating: reduce kP
- If slow to converge after profile: increase kP

# Session: Fix CTRE Pose in Simulation

**Date**: 2026-02-28

## Summary

Removed the `RobotBase.isSimulation()` branching from `getHeading()` and `getPose()` so the CTRE drivetrain pose is used in both real and sim modes. The dead reckoner is kept running in simulation for comparison/debugging.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
  - Removed `RobotBase` import (no longer needed)
  - `getHeading()` — always returns `drivetrain.getState().Pose.getRotation()` instead of branching on sim
  - `getPose()` — always returns `drivetrain.getState().Pose` instead of branching on sim
  - `simulationPeriodic()` — renamed published pose from "CtrePose" to "DeadReckonPose" and updated comment to clarify it's the dead-reckoned comparison value

## Why

The dead reckoner uses instant speed changes (no physics), while the CTRE sim models real motor dynamics. Since teleop uses `getHeading()` for field-to-robot speed conversion, using the dead reckoner heading in sim created a feedback mismatch — the heading used for conversion didn't match the heading the CTRE sim was actually using, causing "FusedPose" and "CtrePose" to drift apart over time.

## Testing Done

- [x] `./gradlew build` — passed, all tests green

## Notes for Next Session

- "FusedPose" now shows the CTRE/fused pose in both sim and real
- "DeadReckonPose" shows the idealized dead-reckoned path in sim for comparison
- The dead reckoner still resets when `resetPose()` is called, keeping it in sync for comparison purposes

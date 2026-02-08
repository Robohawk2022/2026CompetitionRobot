# Session: Update Launcher to 3-Motor Design + SparkMax Onboard PID

**Date**: 2026-02-08

## Summary

Removed the separate intake motor from the launcher (4 motors -> 3). The bottom (lower) wheel now doubles as intake by spinning backward. Migrated wheel velocity control from software PID (50Hz) to SparkMax onboard PID (1kHz). Code-reviewed and fixed critical units mismatch and eject direction bug.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` - Removed `INTAKE_CAN_ID`, `intakePower`, `intakeInverted`. Added `intakeSpeedRPM` (default 1500). kP default changed from 0.001 to 0.0001 (duty cycle units, not volts). Updated comments to document correct units.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java` - Replaced `applyLowerWheelVolts()`/`applyUpperWheelVolts()` with `setLowerWheelRPM()`/`setUpperWheelRPM()`. Removed intake methods. Added `resetPID(kV, kP, kD)`.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java` - Removed intake motor. Uses `feedForward.kV()` (volts/RPM) not `velocityFF()` (duty cycle/RPM). `closedLoop.p()` and `closedLoop.d()` in duty cycle units. `setReference(rpm, kVelocity)` for wheel targets.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java` - Removed intake. Wheels accept RPM targets directly.
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` - Removed software PDController. `driveWheels()` passes RPM to hardware. `applyPIDGains()` pushes live-tuned values at command start. Fixed eject direction (lower wheel forward, not backward).
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` - A=intake, B=eject, X=shoot NEUTRAL, Y=stop.

## Code Review Findings (Fixed)

### CRITICAL: `velocityFF()` vs `feedForward.kV()` units
- `velocityFF()` expects **duty cycle per RPM** (deprecated API)
- `feedForward.kV()` expects **volts per RPM** (correct, matches our Config)
- At 3000 RPM with 0.002 duty_cycle/RPM: `0.002 * 3000 = 6.0` = full power slam
- **Fixed:** switched to `feedForward.kV()` which matches our 0.002 V/RPM

### CRITICAL: kP units mismatch
- `closedLoop.p()` is in **duty cycle per RPM of error**, not volts
- Default 0.001 at 3000 RPM error = 3.0 duty cycle (clamped to 1.0 = full power)
- **Fixed:** default changed to 0.0001, comments updated

### BUG: Eject direction
- Eject had lower wheel backward (same as intake) — ball gets pulled in while agitator pushes back
- **Fixed:** eject now spins lower wheel forward (positive RPM) to push ball out

## Config Changes

- Removed `Launcher/IntakePower%`, `Launcher/IntakeInverted?`, `INTAKE_CAN_ID`
- Added `Launcher/IntakeSpeedRPM` default 1500.0
- Changed `Launcher/Wheels/kP` default from 0.001 to 0.0001 (duty cycle units)
- kV (0.002) unchanged — correct for volts/RPM with `feedForward.kV()` API

## REV SparkMax Closed-Loop Units Reference

| Parameter | API | Units |
|-----------|-----|-------|
| `feedForward.kV()` | new, recommended | volts per RPM |
| `velocityFF()` | deprecated, DO NOT USE | duty cycle per RPM |
| `closedLoop.p()` | - | duty cycle per RPM of error |
| `closedLoop.d()` | - | duty cycle per RPM/s of error change |
| `setReference()` input | - | RPM |
| Closed-loop output | - | duty cycle (-1 to 1) |

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware

## Notes for Next Session

- Tune kV first on real hardware. Start at 0.002 (12V / 5676 RPM NEO free speed). If motor doesn't reach target, increase kV slightly.
- Then add kP (start at 0.0001). If oscillating, decrease kP. If too slow to reach target, increase slightly.
- PIDTestBot.java has a good live-tuning pattern with dashboard-writable values if you need real-time tuning without restarting.
- The `IntakeFrontHardwareSparkMax` uses the old `velocityFF()` API — should also be migrated to `feedForward.kV()` at some point.

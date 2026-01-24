# Session: Add Orbit Drive Command for Reef Scoring

**Date**: 2026-01-23

## Summary

Implemented an "orbit drive" mode that allows the robot to orbit around the 2026 Reef scoring structure while automatically facing it. The driver can control radial (toward/away) and tangential (orbit around) movement using the left stick.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java` - Added `Orbit` interface with configuration parameters:
  - Reef coordinates for Blue/Red alliance (meters)
  - Max radial/tangential speeds (FPS)
  - Heading PID gains (kP, kD, tolerance)
  - Min distance threshold to prevent singularity

- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added:
  - `getOrbitPoint()` - returns alliance-appropriate Reef position
  - `orbitCommand()` - command factory for orbit mode with full turbo/sniper support

- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Added:
  - `orbitCommand(SwerveSubsystem swerve)` - convenience method using controller inputs

- `src/main/java/frc/robot/RobotContainer.java` - Added binding:
  - Left bumper (hold): Activates orbit mode

## Config Parameters Added

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Orbit/BlueReefX_m` | 4.5 | Blue Reef X position (meters) |
| `Orbit/BlueReefY_m` | 4.0 | Blue Reef Y position (meters) |
| `Orbit/RedReefX_m` | 13.1 | Red Reef X position (meters) |
| `Orbit/RedReefY_m` | 4.0 | Red Reef Y position (meters) |
| `Orbit/MaxRadialSpeedFPS` | 8.0 | Max toward/away speed |
| `Orbit/MaxTangentSpeedFPS` | 10.0 | Max orbit speed |
| `Orbit/Heading/kP` | 1.0 | Heading P gain (updated from 0.1) |
| `Orbit/Heading/kD` | 0.005 | Heading D gain |
| `Orbit/Heading/Tolerance` | 2.0 | Heading tolerance (degrees) |
| `Orbit/MinDistance_m` | 0.5 | Minimum orbit radius |

## Commands Added

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `orbitCommand()` | SwerveSubsystem | Orbit around Reef while facing it |

## Controls

- **Left Bumper (hold)**: Activate orbit mode
- **Left Stick Y**: Move toward/away from Reef
- **Left Stick X**: Orbit around Reef (CCW/CW)
- **Right Stick X**: Fine-tune heading (30% trim authority)
- **Triggers**: Turbo/sniper speed modifiers still work in orbit mode

## Math Overview

1. Calculate vector from robot to Reef
2. Compute unit radial and tangent vectors
3. Convert driver stick inputs to radial/tangential velocities
4. Transform to field-relative ChassisSpeeds using rotation matrix
5. Use PD controller to keep robot facing the Reef
6. Apply turbo/sniper speed modifiers

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested with SwerveTestbot
- [x] Radial movement (toward/away from reef) - working
- [x] Tangential movement (orbit around reef) - working
- [x] Heading control (auto-face reef) - working, error < 5°

## Bug Fixes (2026-01-23 Testing Session)

### Heading Control Sign Error
**Problem:** Robot was turning away from the reef instead of toward it.
**Root Cause:** The PD controller call had inverted sign:
```java
// WRONG - gave negative output for positive error
headingPid.calculate(0, -headingError);
// CORRECT - gives positive output for positive error
headingPid.calculate(-headingError, 0);
```
**Fix:** Changed `SwerveSubsystem.java:620` to use correct parameter order.

### Heading kP Too Low
**Problem:** With kP=0.1, the heading control was too slow (~7°/s for 70° error).
**Fix:** Increased default kP from 0.1 to 1.0 in `Config.java` and `networktables.json`.

### Xbox Controller Mapping
**Problem:** Simulation uses Xbox controller axis mapping, but `UseXboxMapping` was false.
**Fix:** Set `Swerve/UseXboxMapping?` to `true` in `networktables.json` for simulation testing.

## Known Issues / TODO

- [ ] Reef coordinates are placeholder values - need actual 2026 field measurements
- [ ] May need further heading PID tuning on real robot
- [x] ~~Test in simulation to verify behavior~~ - DONE

## Notes for Next Session

The orbit command is now tested and working in simulation. Key points:

1. **Heading tracks well** - error typically < 5° while moving
2. **kP=1.0 is stable** - responsive without oscillation
3. **Use Xbox mapping for simulation** - set `Swerve/UseXboxMapping?` to true

Dashboard mode displays: "orbit", "orbit-turbo", or "orbit-sniper" based on current speed mode.

The orbit point coordinates are still placeholders and will need calibration to the actual 2026 Reefscape field layout.

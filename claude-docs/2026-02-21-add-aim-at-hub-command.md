# Session: Add Aim-at-Hub Command with Distance-Based LEDs

**Date**: 2026-02-21

## Summary

Added a teleop aim-at-hub command that auto-rotates the robot to face the alliance hub center while allowing the driver to translate freely. LEDs turn green when in a shooting zone (2.5ft or 3.5ft from hub), red otherwise. Configurable shooting distances and RPMs for each distance.

## Changes Made

### Files Created
- `src/main/java/frc/robot/commands/swerve/SwerveAimAtHubCommand.java` - Teleop command that:
  - Uses PDController to continuously rotate the robot to face the hub
  - Driver left stick controls translation, rotation is automatic
  - LEDs show green when at a shooting distance, red when not
  - Exposes `getRecommendedRPM()` and `inShootingZone()` for future shoot sequencing
  - Uses `Field.getHubCenter()` for alliance-aware hub position

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `SwerveAim` and `Shooting` config interfaces
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added `aimAtHubCommand(LEDSubsystem, GameController)` factory method and imports
- `src/main/java/frc/robot/RobotContainer.java` - Created LED subsystem, wired aim-at-hub to right bumper with LED feedback

## Config Changes

### SwerveAim (heading control while aiming)
- `SwerveAim/kP` default 4.0 — degrees/sec per degree of error
- `SwerveAim/kD` default 0.0
- `SwerveAim/HeadingToleranceDeg` default 2.0
- `SwerveAim/MaxRotateDPS` default 180.0

### Shooting (distance presets and RPMs)
- `Shooting/CloseDistanceFt` default 2.5
- `Shooting/CloseRPM` default 2525.0
- `Shooting/FarDistanceFt` default 3.5
- `Shooting/FarRPM` default 3200.0 (estimated — needs tuning)
- `Shooting/DistanceToleranceFt` default 0.5

## Commands Added

| Command | Subsystem | Description |
|---------|-----------|-------------|
| `aimAtHubCommand(led, controller)` | SwerveSubsystem | Hold right bumper to auto-aim at hub while translating, LED feedback |

## How It Works

1. `Field.getHubCenter()` returns the hub midpoint (red: tags 2+5, blue: tags 18+21)
2. Each 20ms cycle, computes angle from robot pose to hub center
3. PDController drives rotation to minimize heading error
4. Distance is computed via `Util.feetBetween(robotPose, hubCenter)` using odometry
5. LED turns green if within 0.5ft of either shooting distance (2.5ft or 3.5ft)
6. LED turns red if not in either zone
7. All distances, RPMs, and tolerances are tunable in Elastic

## Driver Controls

| Button | Action |
|--------|--------|
| Left bumper (hold) | Orbit mode (existing) |
| **Right bumper (hold)** | **Aim at hub + LED distance feedback** |
| Left stick | Translation (works in aim mode too) |
| Left/right trigger | Sniper/turbo (works in aim mode too) |

## Testing Done

- [x] `./gradlew build` - passed
- [x] All tests passed
- [ ] Simulation tested - not yet
- [ ] Real hardware - not yet

## Known Issues / TODO

- [ ] Tune `SwerveAim/kP` on real robot (4.0 is a starting guess)
- [ ] Tune `Shooting/FarRPM` (3200 is a guess, needs real testing)
- [ ] The hub center AprilTag IDs in `Field.java` may need updating for 2026 field
- [ ] Future: wire `getRecommendedRPM()` into a shoot sequence that sets launcher RPM based on distance
- [ ] Future: consider adding heading-locked indicator (separate from distance LEDs)
- [ ] `LauncherSubsystem` in RobotContainer is declared but never instantiated

## Notes for Next Session

- The `SwerveAimAtHubCommand` exposes `getRecommendedRPM()` which returns the RPM for the current distance zone (close=2525, far=3200, none=0). This can be used to feed into `LauncherSubsystem.shootCommand()` once the launcher is wired up.
- Distance is computed from odometry, which depends on good vision pose estimates. Make sure Limelight is working well before trusting distance values.
- Alliance color is set via Driver Station / FMS. In simulation, reads from FMSInfo NetworkTable.

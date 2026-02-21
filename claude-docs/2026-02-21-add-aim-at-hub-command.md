# Session: Add Aim-at-Hub, Distance LEDs, and Auto-Shoot Command

**Date**: 2026-02-21

## Summary

Added three features:
1. Teleop aim-at-hub command (right bumper) — auto-rotates to face hub while driver translates
2. Always-on LED distance indicator — green in shooting range, red out of range, with alliance flash on startup
3. Auto-shoot command (Y button) — drives to far shooting distance and fires

## Changes Made

### Files Created
- `src/main/java/frc/robot/commands/swerve/SwerveAimAtHubCommand.java` - Teleop aim command with PD heading control

### Files Modified
- `src/main/java/frc/robot/commands/ShootingCommands.java` - Replaced stub with `driveAndShootCommand()` that computes target pose and sequences drive + shoot
- `src/main/java/frc/robot/Config.java` - Added `SwerveAim` and `Shooting` config interfaces
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java` - Added `aimAtHubCommand()` factory
- `src/main/java/frc/robot/subsystems/led/LEDSubsystem.java` - Added always-on distance display, alliance flash on startup/change, `flash()` method for timed overrides
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` - Added `shootAtRPMCommand(DoubleSupplier)` for variable RPM shooting
- `src/main/java/frc/robot/RobotContainer.java` - Wired up launcher, LED, aim command (right bumper), auto-shoot (Y button)

## Config Changes

### SwerveAim
- `SwerveAim/kP` default 4.0
- `SwerveAim/kD` default 0.0
- `SwerveAim/HeadingToleranceDeg` default 2.0
- `SwerveAim/MaxRotateDPS` default 180.0

### Shooting
- `Shooting/CloseDistanceFt` default 2.5
- `Shooting/CloseRPM` default 2525.0
- `Shooting/FarDistanceFt` default 3.5
- `Shooting/FarRPM` default 3200.0 (estimated)
- `Shooting/DistanceToleranceFt` default 0.5

## Commands Added

| Command | Binding | Description |
|---------|---------|-------------|
| `aimAtHubCommand(controller)` | Right bumper (hold) | Auto-aim at hub while translating |
| `driveAndShootCommand(swerve, launcher)` | Y button (press) | Drive to far distance, then shoot |
| `shootAtRPMCommand(rpm)` | (used internally) | Shoot at specific RPM |

## LED Behavior

Priority system (highest first):
1. **Flash** — alliance color on startup/change (3 sec), intake full (2 sec when wired)
2. **Distance** — green when within tolerance of shooting distance, red otherwise

To trigger a flash from any code: `led.flash(LEDSignal.INTAKE_FULL, 2.0)`

## Driver Controls

| Button | Action |
|--------|--------|
| Left bumper (hold) | Orbit mode |
| Right bumper (hold) | Aim at hub + free translation |
| Y (press) | Auto drive-to-distance + shoot |
| Left stick click | Zero pose |
| Right stick click | Reset pose from vision |

## Testing Done

- [x] `./gradlew build` - passed
- [x] All tests passed
- [ ] Simulation tested - partially
- [ ] Real hardware - not yet

## Known Issues / TODO

- [ ] Tune `SwerveAim/kP` on real robot
- [ ] Tune `Shooting/FarRPM` (3200 is a guess)
- [ ] Hub center AprilTag IDs in `Field.java` may need updating for 2026
- [ ] Wire intake stall callback to LED flash (uncomment TODO in RobotContainer)
- [ ] Add alliance preference override for practice without FMS
- [ ] Auto-shoot timeout (3 sec) may need adjusting

## Notes for Next Session

- `ShootingCommands.driveAndShootCommand()` uses `Commands.defer()` to compute the target pose at runtime. It drives along the line from robot to hub, stopping at `farDistance` feet, then shoots at `farRPM`.
- The command is reusable for autonomous — just call `ShootingCommands.driveAndShootCommand(swerve, launcher)`.
- LauncherSubsystem is now fully wired in RobotContainer with sim/real hardware.

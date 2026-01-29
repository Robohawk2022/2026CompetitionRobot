# Session: Refactor SwerveOrbitCommand

**Date**: 2026-01-27

## Summary

Renamed `SwerveOrbitHubCommand` to `SwerveOrbitCommand` and parameterized it to accept a center point. Updated the `orbitCommand()` factory method to return a compound command that first rotates to face the hub, then orbits it.

## Changes Made

### Files Created
- `src/main/java/frc/robot/commands/swerve/SwerveOrbitCommand.java` - Renamed from SwerveOrbitHubCommand, now takes a `Pose2d center` parameter in field coordinates instead of internally calling `Field.getHubCenter()`

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`:
  - Changed import from `SwerveOrbitHubCommand` to `SwerveOrbitCommand`
  - Added import for `Field` utility class
  - Added import for `Commands` (for sequence)
  - Updated `orbitCommand(GameController)` to return a compound command:
    1. Determines hub center using `Field.getHubCenter()`
    2. Calculates heading to face hub from current position
    3. Returns sequence of `SwerveToHeadingCommand` then `SwerveOrbitCommand`

- `src/main/java/frc/robot/testbots/SwerveTestbot.java`:
  - Fixed pre-existing bug: `toPoseCommand` → `driveToPoseCommand` (3 occurrences)

### Files Deleted
- `src/main/java/frc/robot/commands/swerve/SwerveOrbitHubCommand.java` - Replaced by `SwerveOrbitCommand`

## Commands Added/Modified

| Command | Location | Description |
|---------|----------|-------------|
| `SwerveOrbitCommand` | Standalone class | Orbits around a specified center point in field coordinates |
| `orbitCommand()` | SwerveSubsystem | Now returns a compound command: rotate to face hub → orbit hub |

## Key Implementation Details

### SwerveOrbitCommand Constructor
```java
public SwerveOrbitCommand(SwerveSubsystem swerve, GameController controller, Pose2d center)
```
- `center` is the center of rotation in field coordinates
- Each execute cycle converts it to robot-relative for driving

### orbitCommand() Factory Method
```java
public Command orbitCommand(GameController controller) {
    return defer(() -> {
        Pose2d hubCenter = Field.getHubCenter();
        Rotation2d headingToHub = /* calculated from current pose to hub */;
        return Commands.sequence(
            new SwerveToHeadingCommand(this, headingToHub),
            new SwerveOrbitCommand(this, controller, hubCenter)
        );
    });
}
```
- Uses `defer()` to calculate hub position and heading when command is scheduled
- Sequence ensures robot faces hub before starting orbit

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not run this session
- [ ] Real hardware - not tested

## Additional Changes (same session)

### Radial Translation Control
Added right Y joystick control for radial movement (toward/away from center):
- Right X: tangential speed (orbiting around center)
- Right Y: radial speed (closer to or further from center, stick forward = toward)

Implementation details:
- Distance is now updated each cycle (since it changes with radial movement)
- Radial velocity is calculated by normalizing the robot-relative center vector and scaling by speed
- Both radial translation (vx, vy) and rotation are combined in the ChassisSpeeds

Dashboard telemetry updated:
- `LinearSpeed` renamed to `TangentialSpeed`
- Added `RadialSpeed`

## Notes for Next Session

- The orbit command now requires facing the hub first, which adds a small delay before orbiting starts
- Consider adding a parameter to skip the heading phase if the robot is already approximately facing the hub
- Dashboard telemetry renamed from "HubCenter" to "OrbitCenter" in the new command
- Radial and tangential speeds both use `maxOrbit` config - consider separate config values if different speeds are desired

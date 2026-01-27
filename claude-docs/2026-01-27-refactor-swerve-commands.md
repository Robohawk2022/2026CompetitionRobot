# Session: Refactor Swerve Commands

**Date**: 2026-01-27

## Summary

Removed `faceTargetDriveCommand`, eliminated `SwerveTeleopSpeedSupplier` intermediary class, and extracted teleop/orbit command logic into standalone command classes following the guideline that command factory methods should be ~10 lines or less.

## Changes Made

### Files Created
- `src/main/java/frc/robot/commands/swerve/SwerveTeleopCommand.java` - Teleop drive command with sniper/turbo modes
- `src/main/java/frc/robot/commands/swerve/SwerveOrbitCommand.java` - Orbit command with rotation-priority desaturation

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
  - Removed `faceTargetDriveCommand()` method
  - Removed `drivePreserveRotation()` (moved to SwerveOrbitCommand)
  - Removed orbit-related state fields (now in SwerveOrbitCommand)
  - Added `setTelemetry(mode, sniperActive, turboActive)` for commands to update dashboard
  - Added `driveStates(states, speeds)` for commands with custom desaturation
  - Simplified `driveCommand()` and `orbitCommand()` to one-line factory methods
  - Consolidated multiple `driveCommand` overloads into single method
  - Consolidated multiple `orbitCommand` overloads into single method

- `src/main/java/frc/robot/RobotContainer.java`
  - Removed SwerveTeleopSpeedSupplier usage
  - Now uses `swerve.driveCommand(driver)` and `swerve.orbitCommand(driver)` directly
  - Button triggers use `driver.leftBumper()` etc. directly

- `src/main/java/frc/robot/testbots/SwerveTestbot.java`
  - Same changes as RobotContainer

- `src/main/java/frc/robot/testbots/VisionSimTestbot.java`
  - Updated to use new `driveCommand(GameController)` signature

### Files Deleted
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Removed intermediary class

## Architecture Notes

### Command Factory Method Guidelines
Command factory methods in subsystems should be short (~10 lines). If the logic is more complex:
1. Create a standalone command class in `frc.robot.commands.<subsystem>/`
2. The factory method simply returns `new XxxCommand(this, ...)`
3. The command class owns its state and reads config values directly

### SwerveTeleopCommand
- Reads speed limits from Config.Swerve on each execute()
- Applies deadband, axis inversion, input clamping
- Handles sniper/turbo speed modifiers
- Updates subsystem telemetry via `setTelemetry()`

### SwerveOrbitCommand
- Manages its own heading control state (previousHeadingError, initialLockAcquired)
- Includes rotation-priority desaturation logic (drivePreserveRotation)
- Uses `swerve.driveStates()` for custom module state control
- Reads from both Config.Swerve and Config.Orbit

### Subsystem Helper Methods for Commands
Added to SwerveSubsystem to support command classes:
- `setTelemetry(mode, sniperActive, turboActive)` - Update dashboard state
- `driveStates(states, speeds)` - Apply pre-computed module states
- `getTargetPoint()` - Get alliance-specific target position (already existed)

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not tested
- [ ] Real hardware - not tested

## Known Issues / TODO

- None identified

## Notes for Next Session

The command extraction pattern used here can be applied to other subsystems with complex command logic. The key insight is that subsystems provide:
1. Simple factory methods that create command instances
2. Helper methods for commands to update telemetry and drive states
3. State queries (getPose, getHeading, etc.)

Commands own their execution logic and internal state.

# Session: Add CommandLogger Utility Class

**Date**: 2026-01-17

## Summary

Created a CommandLogger utility class that hooks into WPILib's CommandScheduler to log command lifecycle events (initialize, finish, interrupt) with SmartDashboard telemetry.

## Changes Made

### Files Created
- `src/main/java/frc/robot/util/CommandLogger.java` - Utility class that registers with CommandScheduler event hooks to log command events

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added `commandLogging` and `commandExecuteLogging` preferences to `Logging` interface
- `src/main/java/frc/robot/RobotContainer.java` - Added `CommandLogger.enable()` call in constructor

## Config Changes

- Added `Logging/Commands?` with default value `true` - controls init/finish/interrupt logging
- Added `Logging/CommandExecute?` with default value `false` - controls verbose execute logging (rate-limited)

## Features

The CommandLogger provides:
- **Event Logging**: Logs INIT, FINISH, and INTERRUPT events with command name and subsystem requirements
- **Duration Tracking**: Reports how long each command ran when it finishes/interrupts
- **Execute Logging**: Optional rate-limited logging of active commands (every 50 cycles = 1 second)
- **SmartDashboard Telemetry**:
  - `CommandLogger/ActiveCount` - Number of currently running commands
  - `CommandLogger/ActiveCommands` - Comma-separated list of active command names
  - `CommandLogger/TotalRun` - Total commands run this session
  - `CommandLogger/Enabled` - Whether logging is currently enabled

## Log Output Format

```
[CMD] INIT: ArmPresetCommand [ArmSubsystem]
[CMD] FINISH: ArmPresetCommand (ran for 2.34s)
[CMD] INTERRUPT: DriveCommand (ran for 1.50s)
[CMD] EXECUTE: (3 active) IntakeCommand, ShooterSpinUp, DriveCommand
```

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not tested yet
- [ ] Controller triggers - not tested yet

## Notes for Next Session

- Test in simulation by pressing controller buttons to trigger commands
- Verify console output shows `[CMD]` prefixed messages
- Check SmartDashboard for `CommandLogger/` entries
- Toggle `Logging/Commands?` preference to verify logging can be disabled at runtime

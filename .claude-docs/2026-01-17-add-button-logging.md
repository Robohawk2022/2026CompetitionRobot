# Session: Add Button Logging to CommandLogger

**Date**: 2026-01-17

## Summary

Extended CommandLogger to log all Xbox controller button presses, including joystick buttons (L/R stick press), D-pad, bumpers, triggers, and face buttons.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/util/CommandLogger.java` - Added button logging functionality:
  - `addController(String, CommandXboxController)` - Register a controller for tracking
  - `addController(String, XboxController)` - Overload for raw XboxController
  - `pollButtons()` - Call from robotPeriodic() to check for button changes
  - Tracks all 10 Xbox buttons including LStick (index 8) and RStick (index 9)
  - Tracks POV/D-pad with directional names (Up, UpRight, Right, etc.)
  - Tracks triggers as buttons with 0.5 threshold

- `src/main/java/frc/robot/Robot.java` - Added `CommandLogger.pollButtons()` call in robotPeriodic()

- `src/main/java/frc/robot/RobotContainer.java` - Added controller declarations and registration:
  - Added Driver controller on port 0
  - Added Operator controller on port 1
  - Calls `CommandLogger.addController()` for both

## Buttons Logged

| Button | Name in Log |
|--------|-------------|
| A | A |
| B | B |
| X | X |
| Y | Y |
| Left Bumper | LBumper |
| Right Bumper | RBumper |
| Back | Back |
| Start | Start |
| Left Stick Press | LStick |
| Right Stick Press | RStick |
| D-pad | POV-Up, POV-Right, etc. |
| Left Trigger | LTrigger |
| Right Trigger | RTrigger |

## Log Output Format

```
[BTN] Driver A: PRESSED
[BTN] Driver A: RELEASED
[BTN] Operator LStick: PRESSED
[BTN] Driver POV-Up: PRESSED
[BTN] Driver POV-Up: RELEASED
[BTN] Driver LTrigger: PRESSED
```

## Config

Uses existing `Config.Logging.controllerLogging` preference (`Logging/Controller?`, default true)

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not tested yet

## Notes for Next Session

- Button logging respects the existing `Logging/Controller?` preference
- Trigger threshold is 0.5 (configurable via TRIGGER_THRESHOLD constant)
- POV logging tracks direction changes (logs release of previous direction, then press of new direction)

# Session: Add Autonomous Subsystem

**Date**: 2026-01-28

## Summary

Added `AutonomousSubsystem` for PathPlanner-based autonomous program management, along with `DigitBoardProgramPicker` for hardware-based program selection. Fixed several bugs in the initial implementation.

## New Components

### AutonomousSubsystem (`frc.robot.subsystems.auto`)

A subsystem that manages PathPlanner autonomous routines with the following features:

- **Program Selection**: Integrates with `DigitBoardProgramPicker` to allow field operators to select autonomous programs using hardware buttons (or dashboard in simulation)
- **PathPlanner Integration**: Configures `AutoBuilder` with robot config, PID controllers, and named commands
- **Error Recovery**: Wraps PathPlanner initialization in try-catch to prevent robot crashes; falls back to emergency command if configuration fails
- **Alliance Awareness**: Automatically flips paths for red alliance using `Util.isRedAlliance()`
- **Debug Logging**: Optional verbose logging for path following (controlled by `PathPlanner/Debug?` preference)

**Key Methods:**
- `generateCommand()` - Call from `autonomousInit()` to create the selected auto command
- `periodic()` - Updates program selection from DigitBoard

**Customization Points (marked with TODO):**
- `getProgramNames()` - Define short codes and PathPlanner filenames
- `registerNamedCommands()` - Register commands used in auto paths
- `decorateAutoCommand()` - Add pre/post actions to auto routines
- `createEmergencyCommand()` - Fallback if PathPlanner fails
- `createEmergencyStartPose()` - Starting pose if PathPlanner fails

### DigitBoardProgramPicker (`frc.robot.util`)

A hardware-based autonomous program selector using the REV Digit Board:

- **Dual Mode**: Uses DigitBoard hardware on real robot, SendableChooser on dashboard in simulation
- **Button Navigation**: A button scrolls backward, B button scrolls forward through options
- **4-Character Display**: Shows short program codes (e.g., "EX01") on the LED display
- **Wraparound**: Scrolling past the end wraps to the beginning (and vice versa)

**Usage:**
```java
Map<String,String> programs = new LinkedHashMap<>();
programs.put("EX01", "Example1");  // short code -> PathPlanner filename
programs.put("EX02", "Example2");

DigitBoardProgramPicker picker = new DigitBoardProgramPicker("Auto Program", programs);

// In periodic():
String selected = picker.get();  // Returns PathPlanner filename
```

### DigitBoard (`frc.robot.util`)

Low-level driver for REV Digit Board I2C communication:

- 4-character alphanumeric display
- Two buttons (A and B) with press/release detection
- Potentiometer input (note: values may vary)

## Config Changes

Added `Config.PathPlanner` interface:
- `PathPlanner/Translation/kP`, `kI`, `kD` - Translation PID gains (default 0.0)
- `PathPlanner/Rotation/kP`, `kI`, `kD` - Rotation PID gains (default 0.0)
- `PathPlanner/Debug?` - Enable verbose logging (default true)

## Bug Fixes Applied

| File | Issue | Fix |
|------|-------|-----|
| `DigitBoardProgramPicker.java:115` | Button A used `isButtonAPressed()` (continuous) instead of release detection | Changed to `wasButtonAReleased()` |
| `DigitBoard.java:153` | `wasButtonAReleased()` detected press, not release | Fixed logic: `!isPressed && aWasPressed` |
| `DigitBoard.java:172` | `wasButtonBReleased()` same issue | Fixed logic: `!isPressed && bWasPressed` |
| `AutonomousSubsystem.java:71-73` | Exception silently swallowed | Added `Util.log()` and `printStackTrace()` |
| `AutonomousSubsystem.java:170` | `createEmergencyStartPose()` never called | Added `swerve.resetPose()` call in error path |
| `AutonomousSubsystem.java:255` | Typo "mightz" | Fixed to "might" |

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

- PID gains default to 0.0 - must be tuned on real robot
- Named commands are placeholders (`DoSomething1`, etc.) - replace with real subsystem commands
- Program list has example entries - update with actual PathPlanner auto names
- The DigitBoard potentiometer value range is inconsistent - use with caution

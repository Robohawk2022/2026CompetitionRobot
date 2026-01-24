# Session: Fix 8BitDo Ultimate DirectInput Button Mapping

**Date**: 2026-01-23

## Summary

Fixed button mapping for 8BitDo Ultimate controller in DirectInput (D) mode. The controller was incorrectly triggering orbit mode when pressing the Y button because the old mapping assumed XInput mode.

## Problem

The 8BitDo Ultimate controller in DirectInput mode uses different button indices than XInput mode:
- User pressed Y button (physical)
- Controller sent button index 4 (0-indexed) / 5 (1-indexed)
- Old code interpreted button 5 as Left Bumper
- Orbit mode incorrectly activated

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Updated 8BitDo button and axis constants for DirectInput mode

### Button Mapping Changes (WPILib 1-indexed)

| Button | Old Value | New Value | SDL Index |
|--------|-----------|-----------|-----------|
| X | 3 | 4 | b3 |
| Y | 4 | 5 | b4 |
| Left Bumper | 5 | 7 | b6 |
| Right Bumper | 6 | 8 | b7 |
| Back | 7 | 11 | b10 |
| Start | 8 | 12 | b11 |
| Left Stick Click | 9 | 14 | b13 |
| Right Stick Click | 10 | 15 | b14 |

### Axis Mapping Changes

| Axis | Old Value | New Value |
|------|-----------|-----------|
| Left Trigger | 4 | 5 |
| Right Trigger | 5 | 4 |

Note: Triggers are swapped in DirectInput mode compared to XInput.

## Testing Required

- [ ] Y button should NOT trigger orbit mode
- [ ] Left bumper (LB) SHOULD trigger orbit mode
- [ ] Both stick clicks should reset heading
- [ ] Left trigger should activate sniper mode (slow)
- [ ] Right trigger should activate turbo mode (fast)

## Testing Done

- [x] `./gradlew build` - passed

## Sources

- [SDL GameControllerDB - 8BitDo Ultimate mappings](https://github.com/mdqinc/SDL_GameControllerDB)
- [SDL DirectInput support for 8BitDo Ultimate](https://discourse.libsdl.org/t/sdl-added-support-for-the-8bitdo-ultimate-wired-controller-in-directinput-mode-including-the-misc-button-and-paddles/40732)

## Notes

The mapping is based on SDL GameControllerDB entries for the 8BitDo Ultimate controller in DirectInput mode. If the controller is switched to XInput mode (X switch position), the `useXboxMapping` config option should be set to true, which will use the standard Xbox button indices instead.

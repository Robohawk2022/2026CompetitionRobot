# Session: Extract GameController Class

**Date**: 2026-01-26

## Summary

Extracted controller type mapping logic from `SwerveTeleopSpeedSupplier` into a new `GameController` class to centralize controller abstraction. Supports three controller types: 8BitDo (DirectInput), Xbox (XInput), and Logitech. Replaced all `CommandXboxController` usage with `GameController`.

## Changes Made

### Files Created
- `src/main/java/frc/robot/GameController.java` - New controller abstraction that handles mapping between 8BitDo (DirectInput), Xbox (XInput), and Logitech controllers. Auto-detects Logitech by name; uses `Config.SwerveTeleop.useXboxMapping` for 8BitDo vs Xbox

### Files Modified
- `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java` - Simplified to use `GameController`, removed all axis/button mapping constants and logic (delegated to GameController)
- `src/main/java/frc/robot/RobotContainer.java` - Changed from `CommandXboxController` to `GameController`
- `src/main/java/frc/robot/util/CommandLogger.java` - Added overload for `GameController`
- `src/main/java/frc/robot/testbots/SwerveTestbot.java` - Changed to `GameController`
- `src/main/java/frc/robot/testbots/IntakeFrontTestbot.java` - Changed to `GameController`
- `src/main/java/frc/robot/testbots/LEDTestbot.java` - Changed to `GameController`, fixed `robotPeriodic()` to use GameController methods
- `src/main/java/frc/robot/testbots/ShooterTestbot.java` - Changed to `GameController`
- `src/main/java/frc/robot/testbots/ShooterMotorsTestbot.java` - Changed to `GameController`
- `src/main/java/frc/robot/testbots/VisionSimTestbot.java` - Changed to `GameController`, updated drive command to use GameController axis methods
- `src/main/java/frc/robot/testbots/SysIdTestbot.java` - Changed to `GameController`, fixed dashboard to use GameController trigger methods

## GameController API

The new `GameController` class provides:

### Controller Types
```java
public enum Type {
    XBOX,     // Standard Xbox/XInput mapping
    BITDO,    // 8BitDo DirectInput mode
    LOGITECH  // Logitech gamepad (auto-detected by name)
}
```

### Construction
```java
GameController controller = new GameController(0); // port number
```

### Type Detection
- `getType()` - Returns `Type` enum based on controller detection
- Logitech is auto-detected by checking if HID name contains "Logitech"
- Xbox vs 8BitDo determined by `Config.SwerveTeleop.useXboxMapping`

### Axis Methods
- `getLeftX()`, `getLeftY()` - Left stick axes (-1 to 1)
- `getRightX()`, `getRightY()` - Right stick axes (-1 to 1)
- `getLeftTriggerAxis()`, `getRightTriggerAxis()` - Trigger axes (0 to 1, normalized)
- `leftXSupplier()`, `leftYSupplier()`, etc. - DoubleSupplier versions

**Logitech-specific behavior:**
- Joystick axes are inverted (multiplied by -1)
- Triggers are buttons, not axes (reports 0.0 or 1.0)

### Button Triggers
- `a()`, `b()`, `x()`, `y()` - Face buttons
- `leftBumper()`, `rightBumper()` - Bumpers
- `start()`, `back()` - Start/back buttons
- `leftStick()`, `rightStick()` - Stick clicks
- `leftTrigger()`, `rightTrigger()` - Triggers as buttons (threshold 0.5)
- `povUp()`, `povDown()`, `povLeft()`, `povRight()`, `pov(int angle)` - D-pad

### Raw HID Access
- `getHID()` - Returns underlying GenericHID for special cases
- `getPort()` - Returns port number

## Additional Changes (User)

Simplified trigger handling:
- `GameController.leftTriggerSupplier()` and `rightTriggerSupplier()` now return `BooleanSupplier` (true when >50% pressed) instead of `DoubleSupplier`
- `SwerveTeleopSpeedSupplier.sniperTrigger()` and `turboTrigger()` now return `BooleanSupplier`
- `SwerveSubsystem` already expected `BooleanSupplier` for these parameters - no changes needed there
- Config's `sniperFactor` and `turboFactor` remain as `DoubleSupplier` (multiplier values)

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested - not tested
- [ ] Real hardware tested - not tested

## Known Issues / TODO

- [ ] Test with real 8BitDo controller to verify DirectInput mappings still work
- [ ] Test with Xbox controller in simulation
- [ ] Test with Logitech controller to verify button/axis mappings

## Notes for Next Session

The `SwerveTeleopSpeedSupplier` class has been removed (see 2026-01-27-refactor-swerve-commands.md). The `GameController` class handles all controller input abstraction.

Controller type detection:
1. If HID name contains "Logitech" → `Type.LOGITECH`
2. Else if `Config.SwerveTeleop.useXboxMapping` is true → `Type.XBOX`
3. Else → `Type.BITDO`

Logitech-specific quirks:
- Triggers are physical buttons, not analog axes (report 0 or 1)
- Joystick axes report inverted values (corrected in GameController)
- Button numbers differ from Xbox/8BitDo

Trigger flow:
- `GameController.leftTriggerSupplier()` → `BooleanSupplier` (true when >50%)
- Commands use boolean to decide *whether* to apply sniper/turbo
- `Config.SwerveTeleop.sniperFactor`/`turboFactor` provide *how much* to scale

# Session: Extract GameController Class

**Date**: 2026-01-26

## Summary

Extracted controller type mapping logic from `SwerveTeleopSpeedSupplier` into a new `GameController` class to centralize 8BitDo/Xbox controller abstraction. Replaced all `CommandXboxController` usage with `GameController`.

## Changes Made

### Files Created
- `src/main/java/frc/robot/GameController.java` - New controller abstraction that handles mapping between 8BitDo (DirectInput) and Xbox (XInput) controllers based on `Config.Swerve.useXboxMapping`

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

### Construction
```java
GameController controller = new GameController(0); // port number
```

### Axis Methods
- `getLeftX()`, `getLeftY()` - Left stick axes (-1 to 1)
- `getRightX()`, `getRightY()` - Right stick axes (-1 to 1)
- `getLeftTriggerAxis()`, `getRightTriggerAxis()` - Trigger axes (0 to 1, normalized)
- `leftXSupplier()`, `leftYSupplier()`, etc. - DoubleSupplier versions

### Button Triggers
- `a()`, `b()`, `x()`, `y()` - Face buttons
- `leftBumper()`, `rightBumper()` - Bumpers
- `start()`, `back()` - Start/back buttons
- `leftStick()`, `rightStick()` - Stick clicks
- `leftTrigger()`, `rightTrigger()` - Triggers as buttons (threshold 0.5)
- `leftTrigger(double threshold)` - Custom threshold
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

## Notes for Next Session

The `SwerveTeleopSpeedSupplier` class still exists and adds swerve-specific functionality (deadband application, command factories). It now delegates to `GameController` for all raw controller input.

The controller type mapping is determined by `Config.Swerve.useXboxMapping` preference:
- `false` = 8BitDo DirectInput mode (real robot)
- `true` = Xbox XInput mode (simulation/testing)

Trigger flow:
- `GameController.leftTriggerSupplier()` → `BooleanSupplier` (true when >50%)
- `SwerveSubsystem` uses boolean to decide *whether* to apply sniper/turbo
- `Config.sniperFactor`/`turboFactor` (`DoubleSupplier`) provide *how much* to scale

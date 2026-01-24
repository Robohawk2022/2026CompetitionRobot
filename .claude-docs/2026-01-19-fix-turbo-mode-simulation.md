# Session: Fix Turbo Mode in Simulation

**Date**: 2026-01-19

## Summary

Fixed turbo mode not working in simulation. Two issues were identified and resolved:
1. Controller axis mapping was hardcoded for 8BitDo controller (different from Xbox in sim)
2. Turbo speed multiplier was being negated by wheel speed desaturation

## Changes Made

### Files Modified

1. `src/main/java/frc/robot/Config.java`
   - Added `Swerve/UseXboxMapping?` config toggle (default: false)
   - When true, uses Xbox controller axis mapping (for simulation)
   - When false, uses 8BitDo controller axis mapping (for real robot)

2. `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java`
   - Added separate axis constants for 8BitDo and Xbox controllers
   - Updated all supplier methods to check `useXboxMapping` config
   - 8BitDo: triggers on axes 4/5, range -1 to 1, right stick X on axis 2
   - Xbox: triggers on axes 2/3, range 0 to 1, right stick X on axis 4

3. `src/main/java/frc/robot/testbots/SwerveTestbot.java`
   - Replaced hardcoded axis mapping with `SwerveTeleopSpeedSupplier`
   - Now shares same controller logic as `RobotContainer`

4. `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`
   - Added `drive()` overload that accepts custom max speed parameter
   - Updated turbo/sniper drive command to track `effectiveMaxSpeed`
   - Pass `effectiveMaxSpeed` to desaturation so turbo actually exceeds normal max

## Root Causes

### Issue 1: Wrong Axis Mapping
The 8BitDo controller has different axis indices than Xbox:

| Input | 8BitDo | Xbox |
|-------|--------|------|
| Right Trigger | Axis 5 (-1 to 1) | Axis 3 (0 to 1) |
| Left Trigger | Axis 4 (-1 to 1) | Axis 2 (0 to 1) |
| Right Stick X | Axis 2 | Axis 4 |

In simulation, the controller behaves like Xbox, so triggers were reading wrong axes.

### Issue 2: Desaturation Clamping
```java
// Before: turbo multiplied velocity but desaturation clamped it back
double vx = x * maxSpeedMps * turboFactor;  // e.g., 22.5 m/s
desaturateWheelSpeeds(states, maxSpeedMps);  // clamped back to 15 m/s

// After: pass effective max speed to desaturation
double effectiveMaxSpeed = maxSpeedMps * turboFactor;  // 22.5 m/s
desaturateWheelSpeeds(states, effectiveMaxSpeed);      // allows 22.5 m/s
```

## Config Changes

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Swerve/UseXboxMapping?` | false | Set to true for Xbox controller / simulation |

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation tested - turbo mode now increases speed correctly
- [x] TurboActive? indicator shows correctly in Shuffleboard

## Usage

To use turbo in simulation:
1. Run `./gradlew simulateJava` or `./gradlew simulateJava -Probot=SwerveTestbot`
2. In Shuffleboard Preferences, set `Swerve/UseXboxMapping?` to `true`
3. Right trigger activates turbo (1.5x speed by default)

## Notes for Next Session

- Consider auto-detecting simulation vs real robot for controller mapping
- The `useXboxMapping` toggle allows runtime switching without code changes
- Both `RobotContainer` and `SwerveTestbot` now use `SwerveTeleopSpeedSupplier`

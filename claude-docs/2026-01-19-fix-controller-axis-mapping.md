# Session: Fix Controller Input Axis Mapping for 8BitDo

**Date**: 2026-01-19

## Summary

Fixed controller axis mapping for 8BitDo controller. Changed from Xbox controller named methods to raw axis access with explicit axis indices. Also fixed trigger normalization (8BitDo outputs 1 to -1 instead of 0 to 1).

## Problem

1. **Wrong axis mapping**: The `XboxController` named methods assume Xbox controller axis layout where axis 4 is right stick X. On the 8BitDo controller, axes 4 and 5 are the triggers, causing rotation when triggers were pressed.

2. **Different trigger range**: 8BitDo triggers output -1 (not pressed) to 1 (fully pressed), but the code expected 0 to 1.

**8BitDo Controller Axis Layout:**
- Axis 0: Left stick X
- Axis 1: Left stick Y
- Axis 2: Right stick X
- Axis 3: Right stick Y
- Axis 4: Left trigger (-1 to 1)
- Axis 5: Right trigger (-1 to 1)

## Changes Made

### Files Modified

1. `src/main/java/frc/robot/subsystems/swerve/SwerveTeleopSpeedSupplier.java`
2. `src/main/java/frc/robot/testbots/SwerveTestbot.java`

### SwerveTeleopSpeedSupplier Changes

Added axis constants and trigger normalization:
```java
private static final int LEFT_X_AXIS = 0;
private static final int LEFT_Y_AXIS = 1;
private static final int RIGHT_X_AXIS = 2;
private static final int LEFT_TRIGGER_AXIS = 4;
private static final int RIGHT_TRIGGER_AXIS = 5;
```

Trigger normalization formula: `(rawValue + 1) / 2`
- When raw = -1 (not pressed): (-1 + 1) / 2 = 0
- When raw = 1 (fully pressed): (1 + 1) / 2 = 1

### SwerveTestbot Changes

Updated the drive command to use raw axis access with turbo/sniper trigger support and normalization:

```java
swerve.setDefaultCommand(swerve.driveCommand(
        () -> -MathUtil.applyDeadband(controller.getHID().getRawAxis(1), DEADZONE),  // left Y
        () -> -MathUtil.applyDeadband(controller.getHID().getRawAxis(0), DEADZONE),  // left X
        () -> -MathUtil.applyDeadband(controller.getHID().getRawAxis(2), DEADZONE),  // right X
        () -> (controller.getHID().getRawAxis(4) + 1) / 2,  // left trigger = sniper (normalized)
        () -> (controller.getHID().getRawAxis(5) + 1) / 2,  // right trigger = turbo (normalized)
        true));
```

## Control Mapping

| Input | Axis | Range | Function |
|-------|------|-------|----------|
| Left stick Y | 1 | -1 to 1 | Forward/backward |
| Left stick X | 0 | -1 to 1 | Strafe left/right |
| Right stick X | 2 | -1 to 1 | Rotation |
| Left trigger | 4 | -1 to 1 (normalized to 0-1) | Sniper mode (slow) |
| Right trigger | 5 | -1 to 1 (normalized to 0-1) | Turbo mode (fast) |

## Additional Fix: Turbo Speed Multiplier

Fixed turbo mode not actually increasing speed. The issue was that turbo factor was applied to the joystick input before clamping to [-1, 1], which negated the effect at full stick deflection.

**Fix:** Apply turbo/sniper factors to the final velocity (vx, vy) instead of the input (x, y).

**File modified:** `src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java`

Now with turboFactor=1.5, turbo mode will actually be 50% faster than normal max speed.

## Dashboard Indicators

Added boolean indicators to Shuffleboard:
- `Enabled?` - true when robot is enabled
- `TurboActive?` - true when right trigger pressed (> 0.5)
- `SniperActive?` - true when left trigger pressed (> 0.5)

These appear under the Swerve subsystem widget.

## Testing Done

- [x] `./gradlew build` - passed

## Notes

If switching to a different controller in the future:
1. Update axis constants in `SwerveTeleopSpeedSupplier.java`
2. Update raw axis indices in `SwerveTestbot.java`
3. Check trigger range - may need different normalization formula
4. Use Driver Station USB tab to determine correct axis mapping and ranges

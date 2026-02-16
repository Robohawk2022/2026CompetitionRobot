# Session: Rebuild Launcher for New 3-Motor Design

**Date**: 2026-02-16

## Summary

Rebuilt the launcher subsystem from a 2-wheel + agitator design to a new 3-motor design: 2 feeder motors (bottom, facing each other, 12:1 gearbox each) + 1 shooter motor (top, 1:1 gearbox, single NEO). Deleted the agitator subsystem. Extensively PID-tuned all 3 motors. Added LED integration to testbot. Another dev merged cleanup changes mid-session.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/Config.java` — Replaced `Launcher` interface. Removed shot presets, upper/lower wheel config, Agitator interface. Added separate PID for each feeder + shooter, RPM constants, and tunable shooterRPM.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java` — Rewritten for 3 motors: `setFeederLeftRPM`, `setFeederRightRPM`, `setShooterRPM` + getters + separate PID reset methods per motor.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java` — 3 SparkMax motors (CAN 32=feeder left, CAN 11=feeder right, CAN 60=shooter). Another dev refactored: CAN IDs as constructor params, inversion/current limit as class constants, coast mode, updated REV API.
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java` — 3 simulated flywheels with first-order dynamics.
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` — Removed `ShotPreset` enum. New commands: `intakeCommand()`, `ejectCommand()`, `shootCommand()`, `idleCommand()`, `stopCommand()`. Added `getCurrentMode()` getter for cross-package access. Field renamed to `currentShooterRPM` to avoid collision with Config supplier.
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` — Simplified to single subsystem. Added `Preferences.removeAll()` on startup. Added LED integration (green=atSpeed, flashing green=intaking, orange=otherwise).
- `src/main/java/frc/robot/subsystems/led/LEDSignal.java` — Added `INTAKING_ACTIVE(BlinkinCode.SINELON_FOREST)` signal for flashing green during intake.

### Files Deleted
- `src/main/java/frc/robot/subsystems/agitator/AgitatorSubsystem.java`
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardware.java`
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardwareRev.java`
- `src/main/java/frc/robot/subsystems/agitator/AgitatorHardwareSim.java`

## Current Config State (Launcher)

```java
interface Launcher {
    // Separate PID per feeder
    DoubleSupplier feederLeftKV = pref("Launcher/FeederLeft/kV", 0.00017);
    DoubleSupplier feederLeftKP = pref("Launcher/FeederLeft/kP", 0.0004);
    DoubleSupplier feederLeftKD = pref("Launcher/FeederLeft/kD", 0.0);
    DoubleSupplier feederRightKV = pref("Launcher/FeederRight/kV", 0.000175);
    DoubleSupplier feederRightKP = pref("Launcher/FeederRight/kP", 0.0004);
    DoubleSupplier feederRightKD = pref("Launcher/FeederRight/kD", 0.0);
    DoubleSupplier shooterKV = pref("Launcher/Shooter/kV", 0.0002);
    DoubleSupplier shooterKP = pref("Launcher/Shooter/kP", 0.0005);
    DoubleSupplier shooterKD = pref("Launcher/Shooter/kD", 0.0);

    // RPM targets (constants to avoid Preferences override issues)
    double FEEDER_RPM = 2000.0;
    double FEED_SHOOT_RPM = 4000.0;
    double SHOOTER_INTAKE_RPM = 1000.0;

    // Shooter RPM is a Preference for live tuning in Elastic
    DoubleSupplier shooterRPM = pref("Launcher/ShooterRPM", 2525.0);
    DoubleSupplier tolerance = pref("Launcher/ToleranceRPM", 100.0);
}
```

## Hardware Details

- **Feeder gearboxes**: 4:1 * 3:1 = 12:1 total per feeder
- **Shooter**: Single NEO, 1:1 gearbox (dual motor gearbox but only one NEO)
- **Inversion**: Left feeder inverted=true, right feeder inverted=false, shooter inverted=false (hardcoded in LauncherHardwareRev)
- **Current limit**: 60A (set in hardware class)
- **Idle mode**: Coast

## Tuned PID Values

| Motor | kV | kP | kD |
|-------|-----|-----|-----|
| Feeder Left | 0.00017 | 0.0004 | 0.0 |
| Feeder Right | 0.000175 | 0.0004 | 0.0 |
| Shooter | 0.0002 | 0.0005 | 0.0 |

Shooter RPM sweet spot: **2525 RPM**

## Testing Done

- [x] `./gradlew build` — passed
- [x] Extensive PID tuning on real hardware
- [x] LED integration tested in build
- [x] No remaining agitator references

## Key Lessons Learned

1. **Preferences persistence**: `Preferences.initDouble()` only sets if key doesn't exist. Old values on roboRIO override new code defaults. Elastic caches and re-publishes values, defeating `Preferences.removeAll()`. Solution: use hardcoded constants for values that must be controlled by code; only use Preferences for values you actively want to tune live.
2. **NEO kV calculation**: Theoretical kV = 12V / 5676 RPM ≈ 0.00211 V/RPM. But with gearboxes and real-world conditions, actual kV values are much lower (0.00017-0.0002).
3. **High kV causes runaway RPM**: With kV=0.01, a 1000 RPM target produces 10V feedforward (nearly full power), causing motors to vastly overshoot.

## Notes for Next Session

- CAN IDs: Feeder Left = 32, Feeder Right = 11 (same side as shooter), Shooter = 60
- PID gains apply on command start (must release and re-press button to pick up new values)
- RPM targets for feeders read every 20ms from constants
- Shooter RPM reads from Preferences every 20ms (tunable in Elastic as "Launcher/ShooterRPM")
- Multi-ball shooting may need higher current limit or different approach for sustained power
- Launcher not yet wired into RobotContainer (still only in testbot)

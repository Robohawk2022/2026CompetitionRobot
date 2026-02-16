# Session: Add Launcher Subsystem

**Date**: 2026-02-07

## Summary

Created a new LauncherSubsystem with 4 NEO motors (intake, agitator, lower wheel, upper wheel). Different upper/lower wheel speeds control ball arc via the Magnus effect. Wheels use closed-loop velocity control (feedforward + PD feedback). Intake and agitator use open-loop voltage.

## Changes Made

### Files Created
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardware.java` - Hardware interface for 4 motors
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareRev.java` - REV SparkMax implementation
- `src/main/java/frc/robot/subsystems/launcher/LauncherHardwareSim.java` - Simulation with first-order dynamics
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` - Main subsystem with closed-loop wheel control
- `src/main/java/frc/robot/testbots/LauncherTestbot.java` - Standalone test program

### Files Modified
- `src/main/java/frc/robot/Config.java` - Added Launcher config interface

## Config Changes

### Intake & Agitator (open-loop)
- `Launcher/IntakePower%` (default: 30.0) - Intake motor power
- `Launcher/AgitatorPower%` (default: 30.0) - Agitator motor power
- `Launcher/FeedPower%` (default: 50.0) - Agitator power when actively shooting

### Wheel Velocity PID
- `Launcher/Wheels/kV` (default: 0.002) - Feedforward: volts per RPM
- `Launcher/Wheels/kP` (default: 0.001) - Proportional feedback
- `Launcher/Wheels/kD` (default: 0.0) - Derivative feedback
- `Launcher/Wheels/ToleranceRPM` (default: 100.0) - RPM tolerance for atSpeed check

### Shot Presets (RPM targets)
- `Launcher/HighArc/LowerRPM` (default: 2000) / `UpperRPM` (default: 3500)
- `Launcher/Flat/LowerRPM` (default: 3500) / `UpperRPM` (default: 2000)
- `Launcher/Neutral/RPM` (default: 3000)

### Motor Config
- `Launcher/IntakeInverted?` through `Launcher/UpperWheelInverted?` - Motor inversion flags
- `Launcher/CurrentLimit` (default: 40.0A)
- CAN IDs: 30 (intake), 31 (agitator), 32 (lower), 33 (upper) - PLACEHOLDERS

## Commands Added

| Command | Description |
|---------|-------------|
| `idleCommand()` | Stops all motors (default command) |
| `intakeCommand()` | Runs intake + agitator to pull balls in (open-loop) |
| `ejectCommand()` | Reverses intake + agitator to push balls out (open-loop) |
| `spinUpCommand(preset)` | Spins wheels to target RPM (closed-loop, no feed) |
| `shootCommand(preset)` | Spins wheels (closed-loop) + feeds ball (open-loop) |
| `stopCommand()` | Instantly stops all motors |

## Shot Presets

| Preset | Lower Wheel | Upper Wheel | Effect |
|--------|-------------|-------------|--------|
| HIGH_ARC | 2000 RPM | 3500 RPM | Backspin = higher arc (Magnus effect) |
| FLAT | 3500 RPM | 2000 RPM | Topspin = flatter trajectory |
| NEUTRAL | 3000 RPM | 3000 RPM | No spin bias |

## Velocity Control Design

- **Feedforward**: `kV * targetRPM` gets the wheel close to target speed
- **PD feedback**: corrects remaining error without integral windup
- **Dashboard telemetry**: shows current RPM, target RPM, feedforward/feedback/total volts
- **atSpeed()**: returns true when both wheels are within tolerance of their targets
- PID resets in command `initialize()` as per codebase convention

## Tuning Guide

1. Start with `kP = 0` and `kD = 0` (feedforward only)
2. Increase `kV` until wheels get close to target RPM (start ~0.002 for NEO)
3. Add small `kP` to correct steady-state error
4. Only add `kD` if there's oscillation

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware tested

## Known Issues / TODO

- [ ] CAN IDs are placeholders (30-33) - update when build team wires motors
- [ ] All values need tuning on real hardware
- [ ] Motor inversion flags may need toggling depending on wiring
- [ ] Not wired into RobotContainer yet

## Notes for Next Session

- Run testbot: `./gradlew simulateJava -Probot=LauncherTestbot`
- Button A=intake, B=eject, X=high arc, Y=flat, RB=neutral, LB=spin-up only
- Existing ShooterSubsystem and HubberdShooterSubsystem were NOT modified
- Sim motor model is simplified - real PID tuning must happen on hardware

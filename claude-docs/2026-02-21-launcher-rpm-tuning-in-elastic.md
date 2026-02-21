# Session: Make All Launcher RPMs Tunable in Elastic

**Date**: 2026-02-21

## Summary

Converted three static `double` constants in the Launcher config to `DoubleSupplier` prefs so all RPM targets are live-tunable via Elastic/NetworkTables. Added a separate `intakeRPM` pref for the intake motor (CAN 15) during intake mode, defaulting to the same value as feeder RPM.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java` — Replaced 3 static double constants with tunable DoubleSupplier prefs; added `intakeRPM`; reorganized RPM section with intake-mode / shoot-mode comments
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` — Updated 4 command factories to use `.getAsDouble()` suppliers instead of static constants; `intakeCommand` now drives the intake motor (CAN 15) at `intakeRPM` (was 0)

## Config Changes

### Removed
- `double FEEDER_RPM = 2000.0` (static constant, not editable in Elastic)
- `double FEED_SHOOT_RPM = 4000.0` (static constant)
- `double SHOOTER_INTAKE_RPM = 1000.0` (was the shooter speed during intake — removed; shooter is now off during intake)

### Added
- `Launcher/FeederRPM` (default 2000.0) — feeder left + right RPM during intake mode
- `Launcher/IntakeRPM` (default 2000.0) — intake motor (CAN 15) RPM during intake mode; matches feeder RPM by default
- `Launcher/FeedShootRPM` (default 4000.0) — feeder RPM during shoot mode

### Unchanged
- `Launcher/ShooterRPM` (default 2525.0) — shooter RPM during shoot mode
- `Launcher/ShooterIntakeRPM` (default 2525.0) — intake motor RPM during shoot mode
- `Launcher/FeederLeft/kV`, `kP` — feeder left PID gains (individually tunable)
- `Launcher/FeederRight/kV`, `kP` — feeder right PID gains (individually tunable)
- `Launcher/Shooter/kV`, `kP` — shooter PID gains
- `Launcher/ShooterIntake/kV`, `kP` — intake motor PID gains

## Commands Updated

| Command | Before | After |
|---------|--------|-------|
| `intakeCommand()` | feeders @ `FEEDER_RPM`, shooter @ 1000, intake @ 0 | feeders @ `feederRPM`, shooter @ 0, intake @ `intakeRPM` |
| `ejectCommand()` | feeders @ `-FEEDER_RPM` | feeders @ `-feederRPM` |
| `shootCommand()` | feeders @ `FEED_SHOOT_RPM` (static) | feeders @ `feedShootRPM` (tunable) |
| `shootAtRPMCommand()` | feeders @ `FEED_SHOOT_RPM` (static) | feeders @ `feedShootRPM` (tunable) |

## PID Tuning Workflow

All 4 motors have independent kV and kP in Elastic under `Launcher/FeederLeft/`, `Launcher/FeederRight/`, `Launcher/Shooter/`, `Launcher/ShooterIntake/`.

RPM targets update **live** (every 20ms) from Elastic — no command restart needed when changing RPMs.
PID gains require a **command restart** to take effect (applied in `initialize()` via `applyPIDGains()`).

To tune one motor in isolation:
1. Run `intakeCommand` (or `shootCommand`)
2. Set the other motors' RPM prefs to 0 in Elastic
3. Watch the target motor's `Current` vs `Target` on the dashboard
4. Adjust kV and kP, then restart the command to pick up new gains

## Testing Done

- [x] `./gradlew build` — passed, all tests green
- [ ] Simulation tested
- [ ] Real hardware

## Notes for Next Session

- Shooter (CAN 60) no longer spins during `intakeCommand` (was 1000 RPM before). If the shooter needs to help with intake, add a `Launcher/ShooterDuringIntakeRPM` pref and wire it up.
- All 6 RPM prefs are now in Elastic: `FeederRPM`, `IntakeRPM`, `FeedShootRPM`, `ShooterRPM`, `ShooterIntakeRPM`, `ToleranceRPM`

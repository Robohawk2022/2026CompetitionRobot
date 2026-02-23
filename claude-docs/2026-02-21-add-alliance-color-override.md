# Session: Add Alliance Color Override

**Date**: 2026-02-21
**Duration**: ~10 minutes

## Summary

Added an alliance color preference override so teams can set red/blue from Elastic
during practice when no FMS is connected. When the override is enabled, `Util.isRedAlliance()`
returns the configured preference value instead of querying the DriverStation.

## Changes Made

### Files Modified

- `src/main/java/frc/robot/Config.java` - Added `Alliance` interface with two preferences
- `src/main/java/frc/robot/util/Util.java` - Modified `isRedAlliance()` to check override first; added `import frc.robot.Config`

## Config Changes

- Added `Alliance/OverrideEnabled?` with default `false`
- Added `Alliance/IsRed?` with default `true` (red when override is active)

## How to Use

In Elastic (or any NetworkTables dashboard), navigate to Preferences:

1. Set `Alliance/OverrideEnabled?` to `true` to activate the override
2. Set `Alliance/IsRed?` to `true` for Red alliance, `false` for Blue alliance
3. When `Alliance/OverrideEnabled?` is `false`, behavior is unchanged (simulation
   reads from FMSInfo, real robot reads from DriverStation)

## Testing Done

- [x] `./gradlew build` - passed (all 80 existing unit tests still pass)
- [ ] Simulation tested
- [ ] Real hardware tested

## Notes for Next Session

The override check runs before simulation/FMS logic in `isRedAlliance()`, so it works
in both simulation and on-robot. The default is override-off so competition behavior
is unchanged unless explicitly enabled.

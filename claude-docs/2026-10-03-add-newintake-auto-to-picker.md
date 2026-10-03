# Session: Add NEWINTAKE Auto to DigitBoard Picker

**Date**: 2026-10-03
**Duration**: ~5 minutes

## Summary

Registered the new PathPlanner auto `NEWINTAKE` (pushed in commits `2c4ad3e` / `5e426a8`)
in the autonomous program picker under the DigitBoard name `YEET`.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/auto/AutonomousSubsystem.java` - added
  `programs.put("YEET", "NEWINTAKE")` to `getProgramNames()` (Bethesda section)

## Notes

- `NEWINTAKE` was the only `.auto` file not already in the picker.
- It uses the named commands `UntimedIntake`, `WiggleLikeAWorm`, `SitStill` and `Unload`, which are
  all already registered, so no new named commands were needed.
- It references the paths `Copy of DEP4` and `Copy of Depot2`. They work as-is, but renaming them
  in the PathPlanner GUI would make them easier to tell apart (the GUI updates the auto's references too).

## Testing Done

- [x] `./gradlew build` - passed (needed `JAVA_HOME=~/wpilib/2026/jdk`; the shell's JAVA_HOME
  points to a missing Corretto path)
- [ ] Simulation / on-robot run - not done

## Follow-up

- Reviewed which branch changes could affect autos (drive current limits, steer encoder
  RemoteCANcoder, PathPlanner settings.json fixes). No code change needed. The plan is the on-robot
  check in section 4 of `2026-09-21-oct24-robot-checklist.md`.
- Added YEET to that checklist's auto table and to its "at minimum" auto list.

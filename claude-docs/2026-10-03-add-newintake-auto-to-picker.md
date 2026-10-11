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

## PID tuning guide for new developers

- Wrote a shared Claude Doc, "PID Tuning Guide for New Developers":
  https://claude.ai/code/artifact/d8f79e44-7502-41c9-b5cc-1122cedb06e2
- Based on current code, with the recipe adapted from `launcher-guide.md`. That guide is outdated:
  it uses the old 3-motor launcher's `Launcher/Wheels/*` Preference names.
- Found while writing it:
  - The `PathPlanner/Translation/kP` and `PathPlanner/Rotation/kP` Preferences are unused.
    `AutonomousSubsystem` hardcodes 1.0 for both.
  - Shooter kV default (0.0002) is about 1/10 of the 12 V / 5676 RPM estimate, so kP does most of
    the work.
  - Saved Preferences on the roboRIO override new defaults in `Config.java`.

## Follow-up: PathPlanner gains, shooter tune, launcher guide

- PathPlanner path-following P is hardcoded on purpose. Mentor commit `d3adfe2` (2026-03-07)
  commented out the Preferences and set 0.0. `6896ddf` "Auto testing" (2026-03-14) set 1.0, and all
  current autos were tuned with that. The Preference defaults (translation 0.0, rotation 3.0) don't
  match 1.0, so wiring them back up would change auto behavior. Left as-is.
- User confirmed the shooter's kP-heavy tune (kV 0.0002) is intentional: the team couldn't get a
  kV-first tune working. Updated the PID guide doc to say so.
- Rewrote `claude-docs/launcher-guide.md` for the current ShooterSubsystem + BallPathSubsystem
  design (CAN 35 / 8 / 9 / 2, current bindings, Preference names, inversion in code, tolerances).
  Added a warning that `LauncherTestbot` calls `Preferences.removeAll()` on startup.

## Follow-up: feedforward section, guides saved to repo

- Added a "Feedforward: predict first, correct second" section and a control-loop diagram to the
  PID guide doc.
- Saved it to the repo as `claude-docs/pid-tuning-guide.md` (diagram as a Mermaid block).
  The repo copy and the Claude Doc are separate; edit both, or treat the repo copy as the source of truth.
- `launcher-guide.md` now links to the repo copy. The testbot warning now says
  `Preferences.removeAll()` is intentional (user confirmed).
- Added a "Guides for New Developers" section to the CLAUDE.md doc index.

## Follow-up: radio guide

- Added `claude-docs/radio-guide.md` (VH-109 flashing and config, from the WPILib docs) and listed it
  in the CLAUDE.md guide index. The guide tells people never to commit the WPA keys.

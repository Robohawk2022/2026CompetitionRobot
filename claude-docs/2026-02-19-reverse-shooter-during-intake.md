# Session: Reverse Shooter During Intake

**Date**: 2026-02-19

## Summary

Modified the launcher's `intakeCommand()` to reverse the shooter motor at half speed during intake, preventing balls from feeding too far into the shooter.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/launcher/LauncherSubsystem.java` (line 151) — Changed shooter RPM in `intakeCommand()` from `SHOOTER_INTAKE_RPM` (1000 RPM forward) to `-SHOOTER_INTAKE_RPM / 2.0` (500 RPM reverse)

## Testing Done

- [ ] `./gradlew build` — failed due to pre-existing Gradle/JDK compatibility issue (`Type T not present` in build.gradle line 78), unrelated to this change
- [ ] Simulation not tested — same build issue prevents simulation

## Notes for Next Session

The Gradle build has a pre-existing issue at build.gradle line 78 (`Type T not present` error during task creation). This needs to be resolved separately — likely a JDK version mismatch with the Gradle version (8.11).

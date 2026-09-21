# Session: Add Drive Current Limits and On-Robot Logging

**Date**: 2026-09-21
**Duration**: ~30 minutes

## Summary

Found that the swerve **drive** motors had no current limit of any kind, and that
nothing was writing telemetry to disk. Both are fixed. This is prep for the Oct 24
preseason event, which is being used as a diagnostic session for last season's
intermittent brownouts.

## The bug

`TunerConstants.driveInitialConfigs` was a bare `new TalonFXConfiguration()`. The steer
motors got a 60A stator limit right below it; the drive motors got nothing. Phoenix 6
defaults to **no** current limit, so four Krakens were free to pull whatever they wanted
off the bus.

What hid it: `SwerveHardwareConfig.DRIVE_CURRENT_LIMIT_AMPS = 60.0` existed and looked
like the limit. Its only consumer was PathPlanner's `ModuleConfig` in the `catch` branch
of `AutonomousSubsystem` (the fallback that only runs if `RobotConfig.fromGUISettings()`
throws). It is an input to path-planning math and never reached a motor controller.
Grepping for the constant made it look like we were covered.

Separately, `SignalLogger.start()` was never called (the `SignalLogger` import was only
used inside SysId callbacks writing to a logger that was never started) and
`DataLogManager` was not used at all. The per-module `DriveAmps/*` telemetry in
`SwerveSubsystem` was real but live-only — the DS log would have shown the brownout
without showing what caused it.

## Changes Made

### Files Modified
- `subsystems/swerve/SwerveHardwareConfig.java` - replaced the ambiguous
  `DRIVE_CURRENT_LIMIT_AMPS` with four clearly-named constants:
  `DRIVE_SUPPLY_CURRENT_LIMIT_AMPS` (60), `DRIVE_SUPPLY_LOWER_LIMIT_AMPS` (40),
  `DRIVE_SUPPLY_LOWER_TIME_SEC` (1.0), `DRIVE_STATOR_CURRENT_LIMIT_AMPS` (120)
- `subsystems/swerve/TunerConstants.java` - `driveInitialConfigs` now sets a supply
  current limit; `kSlipCurrent` now reads `DRIVE_STATOR_CURRENT_LIMIT_AMPS` instead of a
  hardcoded 120 so there is one source of truth
- `subsystems/auto/AutonomousSubsystem.java` - PathPlanner's fallback `ModuleConfig` now
  uses `DRIVE_STATOR_CURRENT_LIMIT_AMPS`, which is the limit its torque model actually
  wants
- `Robot.java` - added `DataLogManager.start()`, `DriverStation.startDataLog(...)` and
  `SignalLogger.start()` in the constructor
- `RobotContainer.java` - uncommented `PowerSubsystem` (PDH monitoring) and added a note
  that the channel map keys are placeholders

## Why supply and not stator

Supply current is what pulls the battery down, and CTRE does not derive it from anything
else, so setting it in `driveInitialConfigs` will stick. The stator side is set by CTRE
from `withSlipCurrent()`, so configuring a stator limit in the same place risks being
silently overwritten — the slip current constant is the correct knob for it.

Effective behavior: each drive motor may draw up to 60A supply for 1 second, then drops
to 40A sustained. 4 x 40A = 160A sustained drivetrain draw, which a healthy battery
holds up fine.

## Testing Done

- [x] `./gradlew build` - passed, all UtilTest tests pass
- [ ] Simulation - not run (current limits are a no-op in sim)
- [ ] Real hardware - **not tested**, see below

## Known Issues / TODO

- [ ] **Verify on the real robot before Oct 24.** Confirm in Tuner X that the drive
      motors report a supply limit of 60/40A and a stator limit of 120A. The Phoenix6
      jar was not available locally to confirm exactly how CTRE merges
      `DriveMotorInitialConfigs` with slip current.
- [ ] 60/40A is a conservative starting point, not a tuned value. Drivers may notice
      reduced acceleration. Raise it only with log data in hand.
- [ ] `PDH_CAN_ID = 1` and the `PowerSubsystem` channel map are still unverified against
      real wiring. The map keys are mechanism CAN IDs used as placeholders; they need to
      be actual PDH channel numbers before the per-channel breakdown is meaningful.
      Voltage and total current are correct regardless.
- [ ] `SignalLogger` writes continuously and will fill roboRIO storage over time. Plug in
      a USB stick for the event and clear logs afterward.
- [ ] `TunerConstants.java:51` uses `SteerFeedbackType.FusedCANcoder`, which is a Phoenix
      Pro feature. We did not buy the $150/season pass, so we have likely been silently
      falling back to unfused remote CANcoder all season. Worth confirming — it affects
      both steering accuracy and next year's budget.

## Notes for Next Session

Context: this came out of the Sep 2026 email thread about whether to reuse the Mk5n
modules or buy new REV MAXSwerve. The brownouts were being cited as evidence that
Krakens draw too much. They are equally consistent with running eight unlimited Krakens
on tired batteries, which is what we actually did. The Oct 24 event is meant to settle
it with data; these changes are what make that data readable.

After the event, read the `.hoot` files in Tuner X (per-module supply current vs time)
alongside the DS log (battery voltage vs time). If voltage sags with low drive current,
it's the batteries. If drive current spikes right before the sag, the limits need to come
down further.

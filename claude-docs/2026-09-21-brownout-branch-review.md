# Session: Brownout Branch Review

**Date**: 2026-09-21
**Duration**: ~60 minutes

## Summary

Reviewed Jay's `brownout-fixes-oct24` branch and the email thread with Ed, and
cherry-picked the changes that hold up onto `bethesda-auto` by hand. Both branches
fork from `c34fe6e`, so the brownout branch has none of the Bethesda auto/path work
and could not be merged as-is.

## Facts verified

- Phoenix 6 26.1.1 `CurrentLimitsConfigs` defaults: stator 120A on, supply 70A on,
  lower limit 40A after 1.0s. Drive motors were never unlimited.
- The CTRE `SwerveModule` writes `SlipCurrent` into the drive motor's stator limit and
  torque-current peaks, overriding anything in `driveInitialConfigs`. The supply limits
  set there do survive.
- All driving (teleop, PathPlanner, jiggle) goes through `SwerveRequest.RobotCentric`,
  which defaults to open-loop voltage. A drive open-loop ramp would therefore affect
  autos too, and a closed-loop ramp would do nothing.
- Every PathPlanner path caps itself at 3.0 m/s and 3.0 m/s^2 (two slower paths), well
  below both the 5.29 m/s top speed and the torque limit at 80A, so the settings.json
  changes should not visibly change auto trajectories.

## Changes Made

### Files Modified
- `swerve/SwerveHardwareConfig.java` - drive stator 120 -> 80A, drive supply lower
  time 1.0 -> 0.5s, new steer supply limits 40/25A after 0.5s; corrected comments
- `swerve/TunerConstants.java` - steer supply limits; `FusedCANcoder` ->
  `RemoteCANcoder` (no Phoenix Pro license); corrected comments
- `deploy/pathplanner/settings.json` - `driveCurrentLimit` 60 -> 80 (matches slip
  current), `maxDriveSpeed` 5.45 -> 5.29 (matches code), `frModuleX` 0.24 -> 0.245 (typo)
- `commands/swerve/SwerveTeleopCommand.java` - rotation is recomputed every loop; before,
  turning off `SwerveTeleop/SniperRotation?` froze rotation at its last value
- `RobotContainer.java` - PDH channel map was keyed by CAN IDs (mislabelling real PDH
  slots 2, 8, 9); replaced with an empty `PDH_CHANNEL_NAMES` to fill from wiring
- `Robot.java` - joystick data in the DS log; `PowerLog` start/update
- `swerve/SwerveSubsystem.java` - TalonFX current/voltage signals at 20 Hz;
  `DriveStatorAmps/*` on the dashboard
- `shooter/ShooterSubsystem.java` - fixed stale javadoc on `idleCommand()`
- `power/*` - PDH brownout / sticky brownout faults (from the brownout branch)
- `claude-docs/2026-09-21-add-drive-current-limits-and-logging.md` - correction note

### Files Created
- `util/PowerLog.java` - (from the brownout branch) battery volts, brownout count and
  CAN bus stats written straight to the wpilog every loop

## Rejected from the brownout branch (and why)

| Change | Reason |
|--------|--------|
| Drive open/closed-loop ramps | Affects autos and jiggle; closed-loop ramp is a no-op |
| Joystick SlewRateLimiter | Ed: needs drive-team time to tune |
| SPARK ramps (ballpath/shooter) | Smart current limits already cap current; ramp just adds lag |
| Only push SPARK PID on change | Gains aren't persisted; after a SPARK brownout reset they'd never be restored |
| Digit Board write-on-change | Ed tried it; display blanks without constant writes |
| Shooter idle 0 rpm at enable | Reverts Ed's deliberate 2026-03-04 change |
| Weaker jiggle / X blocked during shoot | Autos jiggle while shooting; would change tuned autos |

## Testing Done

- [x] `./gradlew build` - passed
- [x] Simulation - started and ran ~2 min, no exceptions
- [ ] Real hardware - not tested

## Known Issues / TODO

- [ ] Before Oct 24: drive with the new limits and run the autos once to confirm
- [ ] Confirm in Tuner X: drive 80A stator / 60->40A supply, steer 60A stator /
      40->25A supply, and the unlicensed-feature fault is gone
- [ ] Fill in `PDH_CHANNEL_NAMES` from the PDH wiring
- [ ] Plug a USB stick into the roboRIO for the event (wpilog + hoot)
- [ ] `WHEEL_COF = 2.255` still unverified; typical tread on carpet is ~1.1-1.3
- [ ] Ed wants CLAUDE.md deleted and rewritten from scratch (smaller)

## Notes for Next Session

Oct 24 is a diagnostic event: keep behavior changes minimal so the logs are readable.
After it, compare `.hoot` drive supply current against `/power/batteryVolts` in the
wpilog. Revisit ramps only if the logs show current spikes lining up with sags.

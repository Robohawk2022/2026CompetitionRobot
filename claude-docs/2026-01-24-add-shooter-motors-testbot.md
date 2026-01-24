# Session: Add Shooter Motors Testbot

**Date**: 2026-01-24

## Summary

Created a new testbot with three SparkMax motors (intake, index, shoot) with two button-triggered functions that run the motors at configurable percentages.

## Changes Made

### Files Created
- `src/main/java/frc/robot/testbots/ShooterMotorsTestbot.java` - Testbot with three SparkMax motors

### Files Modified
- None (removed temporary Config.java changes)

## Button Mappings

| Button | Function | Intake | Index | Shoot |
|--------|----------|--------|-------|-------|
| A | Intake | 60% CCW | 5% CW | off |
| X | Shoot | off | 60% CW | 60% CCW |

## Dashboard Configuration

All percentages are editable via SmartDashboard under "ShooterMotorsTestbot":
- `IntakeFunc/IntakePct` - Intake motor % for intake function (default: 60)
- `IntakeFunc/IndexPct` - Index motor % for intake function (default: 5)
- `ShootFunc/IndexPct` - Index motor % for shoot function (default: 60)
- `ShootFunc/ShootPct` - Shoot motor % for shoot function (default: 60)

Also displays read-only current readings (amps) for each motor.

## CAN IDs

- Intake: 20
- Index: 21
- Shoot: 22

## Testing Done

- [x] `./gradlew build` - passed

## Notes for Next Session

- CAN IDs (20, 21, 22) may need to be updated to match actual hardware
- Positive voltage = clockwise, negative = counter-clockwise (may need adjustment based on motor orientation)
- Run with: `./gradlew simulateJava -Probot=ShooterMotorsTestbot`

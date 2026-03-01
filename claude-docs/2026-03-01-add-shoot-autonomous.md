# Session: Add Shoot Autonomous Program

**Date**: 2026-03-01

## Summary

Added a code-based autonomous program ("SHT" / "SHOOT") that drives to 8 feet from the alliance hub and shoots all balls. Uses the existing `ShootingCommands.driveAndShootCommand()` which handles orient + spin-up + feed.

## Changes Made

### Files Modified
- `src/main/java/frc/robot/subsystems/auto/AutonomousSubsystem.java` - Added shooter, ballPath, LED subsystem parameters; added "SHOOT" as a code-based auto option
- `src/main/java/frc/robot/RobotContainer.java` - Pass all subsystems to AutonomousSubsystem; moved auto creation after all subsystems
- `src/main/java/frc/robot/testbots/AutonomousTestbot.java` - Updated to match new constructor signature

## How It Works

1. Select "SHT" on the DigitBoard program picker
2. When autonomous starts, `driveAndShootCommand` runs:
   - **Phase 1**: Drives to 8 feet from hub (along line from current position to hub center, facing the hub) while spinning up shooter
   - **Phase 2**: Feeds balls into the shooter while keeping shooter spinning
3. The shoot distance is configurable via `BallHandling/ShootDistance` preference (default: 8.0 feet)

## Testing Done

- [x] `./gradlew build` - passed
- [ ] Simulation tested
- [ ] Real hardware

## Notes for Next Session

- The auto uses `orientToShoot()` which computes the target pose dynamically based on current robot position relative to the hub
- Hub center is alliance-aware (blue: midpoint of tags 18+21, red: midpoint of tags 2+5)
- The shoot distance can be tuned via Preferences without code changes

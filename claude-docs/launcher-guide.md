# Launcher Guide (Shooter + Ball Path)

> Rewritten 2026-10-03. The original guide (2026-02-08) described the old 3-motor
> `LauncherSubsystem` with upper/lower wheels and arc presets. That design was split into
> `ShooterSubsystem` + `BallPathSubsystem` on 2026-02-28
> (see `2026-02-28-split-launcher-into-shooter-ballpath.md`). The old `Launcher/*`
> Preferences no longer exist.

## Overview

The launcher is two subsystems with **4 NEO motors**, all on SPARK MAX controllers using
onboard velocity PID:

| Subsystem | Motor | CAN ID | Purpose |
|-----------|-------|--------|---------|
| `ShooterSubsystem` | Shooter | 35 | Flywheel that launches balls |
| `BallPathSubsystem` | Intake | 8 | Pulls balls in from the field |
| `BallPathSubsystem` | Feeder | 9 | Moves balls toward the shooter |
| `BallPathSubsystem` | Agitator | 2 | Pushes balls from the hopper into the feeder when shooting |

CAN IDs are constants in `RobotContainer`; `LauncherTestbot` reuses them. Inversion and
current limits (40 A) are constants in `ShooterHardwareRev` / `BallPathHardwareRev`, not
Preferences. The intake motor is inverted relative to the feeder and agitator in code.

### How it works

**Intake** (`ShootingCommands.intakeMode`):
1. Spin the shooter to intake speed (`Shooter/IntakeRpm`, 1000) and wait until it's at speed.
2. Run intake + feeder at `BallHandling/IntakeSpeeds/*` (agitator off) while the shooter holds 1000 RPM.

**Shoot** (`ShootingCommands.shootMode`):
1. Spin the shooter to `Shooter/ShootRpm` (3300) and start intake + feeder at feed speeds,
   agitator off. Wait until the shooter is at speed, plus 0.5 s.
2. Feed: intake, feeder and agitator at `BallHandling/FeedSpeeds/*` until the trigger is released.

**Eject** (`ballPath.ejectCommand()`): intake, feeder and agitator run backwards at
`BallHandling/EjectSpeeds/*` to clear a jam. The shooter is not involved.

When nothing is running, the ball path coasts and the shooter idles at intake speed
(1000 RPM) from the moment the robot is enabled.

### Controls

| Competition robot (driver) | LauncherTestbot | Action |
|---|---|---|
| Left trigger (hold) | A (hold) | Intake |
| Right trigger (hold) | X (hold) | Shoot |
| B (hold) | B (hold) | Eject |
| — | Y (press) | Stop everything |
| — | Left bumper (hold) | Run one motor at `LauncherTestbot/TestRpm` (pick the motor in code) |

---

## Testing Motor Identity and Direction

### What you need

- Robot on blocks or held securely (rollers and flywheel will spin)
- Laptop connected to the robot
- Elastic or Shuffleboard open
- REV Hardware Client (optional, for checking CAN IDs)

### Step 1: Verify CAN IDs

Use **REV Hardware Client** to confirm each SPARK MAX has the ID in the table above.

### Step 2: Deploy the testbot

```bash
./gradlew deploy -Probot=LauncherTestbot
```

> **Warning:** `LauncherTestbot` calls `Preferences.removeAll()` at startup on purpose, so
> the code defaults always win. On the competition robot this **erases every saved Preference**,
> including any gains or speeds tuned from the dashboard. Copy any tuned values into
> `Config.java` first, and redeploy the competition code afterwards.

### Step 3: Check each motor

Watch these on the dashboard:
- `ShooterSubsystem/ShooterMotor/CurrentRpm`, `DesiredRpm`, `AtSpeed?`
- `BallPathSubsystem/IntakeMotor/CurrentRpm` (same for `FeederMotor`, `AgitatorMotor`)
- `.../Amps` for each motor

**Intake (A):** the shooter spins up first, then intake and feeder pull the ball in. The agitator stays off.

**Shoot (X):** the shooter spins to 3300, then all three ball-path motors push the ball into it.

**Eject (B):** all three ball-path motors push the ball back out.

To test **one motor at a time**, hold the left bumper. By default it runs the intake at
`LauncherTestbot/TestRpm` (1000, editable on the dashboard). To test a different motor,
uncomment its line in `LauncherTestbot.java` and redeploy.

### If a motor spins the wrong way

Inversion is in code, not Preferences. Change it in `ShooterHardwareRev.INVERTED`, or in
the `createMotor(...)` calls in the `BallPathHardwareRev` constructor. Then redeploy.

If the wrong motor spins, two CAN IDs are swapped. Fix it in REV Hardware Client, or
change the constants in `RobotContainer`.

---

## PID Tuning

For the full walkthrough, including what feedforward is, see
[`pid-tuning-guide.md`](pid-tuning-guide.md).

### Gains

Every motor has its own gains, declared as `PIDFConfig` in `Config.java`:

| Motor | Preference prefix | kP | kV |
|-------|-------------------|----|----|
| Shooter | `ShooterSubsystem/ShooterMotor/` | 0.002 | 0.0002 |
| Intake | `BallPathSubsystem/IntakeMotor/` | 0.00004 | 0.00017 |
| Feeder | `BallPathSubsystem/FeederMotor/` | 0.00004 | 0.00017 |
| Agitator | `BallPathSubsystem/AgitatorMotor/` | 0.00004 | 0.00017 |

Each prefix also has `kI`, `kIZone` and `kD`. All are 0. Leave kI at 0 (integral windup).

**The shooter is tuned mostly on kP on purpose.** By the usual rule, kV would start near
12 V / 5676 RPM ≈ 0.0021. The team couldn't get a kV-first tune to work on the shooter and
settled on the current values, which the autos were tuned with. Don't change the shooter
gains without a mentor and a re-check of the autos.

### Tuning order (for a new or re-tuned motor)

1. Set kP and kD to 0.
2. Tune **kV** until CurrentRpm reaches about 80–95% of DesiredRpm.
3. Add **kP**, starting around 0.0001 and doubling, until the motor reaches the goal and holds steady.
4. Add a tiny **kD** (0.00001) only if kP alone overshoots.

REV gains are very small. A kP of 0.001 with a 1000 RPM error is already full output.

### Live tuning workflow

1. Change the gain in Elastic → **Preferences**.
2. **Release the button and press it again.** Gains are only sent to the SPARK when a
   shooter or ball-path command starts (`resetPid()`).
3. Graph `CurrentRpm` against `DesiredRpm` and repeat.
4. When it's good, copy the values into `Config.java`. The `PIDFConfig` argument order is
   **kP, kI, kIZone, kD, kV**.

Saved Preferences override new code defaults. After changing a default in `Config.java`,
delete the old key from Preferences on the robot, or it will keep the saved value.

### At-speed tolerances

| Check | Tolerance | Where |
|-------|-----------|-------|
| Shooter `AtSpeed?` | ±100 RPM | `Shooter/ShootSpeedTolerance` (Preference) |
| Ball path `AtSpeed?` (intake + feeder) | ±50 RPM | hardcoded in `BallPathSubsystem.MotorStatus.atSpeed()` |

If `AtSpeed?` flickers, the tolerance is too tight or the gains are too low.

### Troubleshooting

| Problem | Likely cause | Fix |
|---------|-------------|-----|
| Motor doesn't spin at all | Wrong CAN ID, or kV and kP both 0 | Check REV Hardware Client and Preferences |
| Shoot never starts feeding | Shooter never reaches `AtSpeed?` | Check shooter CurrentRpm vs DesiredRpm; check battery |
| RPM oscillates around target | kP too high | Reduce kP, or add a small kD |
| Reaches ~80% but not target | kP or kV too low | Raise kP slightly |
| Changed a gain, nothing happened | Command didn't restart | Release and press the button again |
| `Stalled?` true | Jam, or motor blocked | Clear the jam (eject); stall threshold is `Shooter/StallSpeed` / `BallHandling/StallSpeed` |
| Motor gets hot | Long runs near the 40 A current limit | Check for jams and friction; reduce RPM targets |

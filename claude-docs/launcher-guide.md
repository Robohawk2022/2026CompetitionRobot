# Launcher Subsystem Guide

## Overview

The launcher has **3 NEO motors** controlled by SparkMax controllers:

| Motor | CAN ID | Control | Purpose |
|-------|--------|---------|---------|
| Agitator | 31 | Open-loop voltage | Feeds balls from intake toward the wheels |
| Lower wheel | 32 | Closed-loop velocity (SparkMax PID) | Bottom flywheel, doubles as intake |
| Upper wheel | 33 | Closed-loop velocity (SparkMax PID) | Top flywheel |

All motors are mounted on the **left side** of the robot (from the ball's perspective facing the robot).

### How it works

```
    Ball enters (intake side) ────────────── Ball exits (shot)

         ╭───╮
         │ U │  Upper wheel (CAN 33)
         ╰───╯
      ~~ ball ~~
         ╭───╮
         │ L │  Lower wheel (CAN 32)
         ╰───╯

    Agitator (CAN 31) sits between intake opening and the wheels,
    pushing balls toward or away from the wheel pair.
```

### Commands

| Button | Command | Lower Wheel | Upper Wheel | Agitator |
|--------|---------|-------------|-------------|----------|
| A (hold) | Intake | Backward (-RPM) | Off | Forward (feeds toward wheels) |
| B (hold) | Eject | Forward (+RPM) | Off | Backward (pushes back to intake side) |
| X (hold) | Shoot NEUTRAL | Forward (3000 RPM) | Forward (3000 RPM) | Forward (feeds into wheels) |
| Y (press) | Stop | Off | Off | Off |

- **Intake**: Lower wheel spins backward to pull balls in from the field. Agitator helps feed the ball inward.
- **Eject**: Reverses everything to push the ball back out through the intake side (for clearing jams).
- **Shoot**: Both wheels spin forward at high RPM. Agitator feeds the ball into the spinning wheels, which launch it out.

### Arc control (shot presets)

By running the upper and lower wheels at different speeds, the ball gets spin (Magnus effect):

| Preset | Lower RPM | Upper RPM | Effect |
|--------|-----------|-----------|--------|
| HIGH_ARC | 2000 | 3500 | Upper faster = backspin = ball curves up |
| FLAT | 3500 | 2000 | Lower faster = topspin = flat trajectory |
| NEUTRAL | 3000 | 3000 | Equal speed = no spin bias |

---

## Testing Motor Identity and Direction

### What you need

- Robot on blocks or securely held (wheels will spin)
- Laptop connected to robot (USB or WiFi)
- Elastic or Shuffleboard open
- REV Hardware Client (optional, for identifying CAN IDs)

### Step 1: Verify CAN IDs

Before running any code, make sure each SparkMax is flashed with the correct CAN ID:

| Motor | Expected CAN ID |
|-------|-----------------|
| Agitator | 31 |
| Lower wheel | 32 |
| Upper wheel | 33 |

Use **REV Hardware Client** to connect to each SparkMax individually and verify/set the CAN ID.

### Step 2: Test one motor at a time

Deploy the LauncherTestbot:
```bash
./gradlew deploy -Probot=LauncherTestbot
```

Open Elastic/Shuffleboard and watch:
- `LauncherSubsystem/Mode`
- `LauncherSubsystem/LowerWheelCurrent`
- `LauncherSubsystem/UpperWheelCurrent`
- `LauncherSubsystem/AgitatorRPM`

**Test intake (A button):**
- Only the lower wheel and agitator should spin
- If the upper wheel spins instead, the CAN IDs for lower and upper are swapped
- The lower wheel should pull the ball IN (toward the robot). If it pushes OUT, set `Launcher/LowerWheelInverted?` to `true`
- The agitator should push the ball toward the wheels. If it pushes backward, set `Launcher/AgitatorInverted?` to `true`

**Test shoot (X button):**
- Both wheels should spin
- Both wheels should push the ball toward the EXIT (out of the robot)
- Since the motors are on the same side, one wheel needs to be inverted relative to the other
- If a wheel pushes the ball the wrong way, flip its `Inverted?` preference

### Step 3: Set inversion preferences

Open Shuffleboard Preferences and adjust:

| Preference | Set to `true` if... |
|------------|---------------------|
| `Launcher/LowerWheelInverted?` | Lower wheel pushes ball the wrong way when shooting |
| `Launcher/UpperWheelInverted?` | Upper wheel pushes ball the wrong way when shooting |
| `Launcher/AgitatorInverted?` | Agitator pushes ball away from wheels during intake |

**Key rule:** Since both wheels are on the same side, one of `LowerWheelInverted?` or `UpperWheelInverted?` will be `true` and the other `false`. Both wheels must push the ball toward the exit when shooting.

### Step 4: Verify all commands

After setting inversions, test each button and confirm:

1. **A (intake):** Ball gets pulled into the robot
2. **B (eject):** Ball gets pushed back out the intake side
3. **X (shoot):** Ball launches out the exit at speed
4. **Y (stop):** Everything stops

---

## PID Tuning

The wheels use SparkMax **onboard PID** running at 1kHz. There are three gains to tune:

| Gain | Preference | Units | Purpose |
|------|-----------|-------|---------|
| kV | `Launcher/Wheels/kV` | volts / RPM | Feedforward: baseline voltage to reach target speed |
| kP | `Launcher/Wheels/kP` | duty cycle / RPM error | Proportional: corrects remaining error |
| kD | `Launcher/Wheels/kD` | duty cycle / (RPM/s) error | Derivative: dampens oscillation |

### Tuning order

**Always tune in this order: kV first, then kP, then kD (if needed).**

#### 1. Tune kV (feedforward)

kV is the most important gain. It tells the motor how much voltage to apply per RPM of target speed. This gets you close to the target without any feedback.

**Starting value:** `12.0 / 5676 = 0.00211` (12V battery / NEO free speed)

**Procedure:**
1. Set `kP` to `0` and `kD` to `0` (feedforward only)
2. Set `kV` to `0.002`
3. Hold X to shoot at NEUTRAL (3000 RPM default)
4. Watch `LowerWheelCurrent` and `UpperWheelCurrent` on the dashboard
5. The wheels won't quite reach 3000 RPM (there's friction and load) — that's expected
6. If the wheels reach ~80-90% of the target, kV is good. Move on to kP.
7. If the wheels are way too slow, increase kV slightly (e.g., 0.0025)
8. If the wheels overshoot, decrease kV slightly (e.g., 0.0018)

**What to watch for:**
- Wheels should spin up smoothly, not jerk
- Current draw should be reasonable (watch `LowerWheelAmps`, `UpperWheelAmps` — should be under the 40A limit)

#### 2. Tune kP (proportional)

kP corrects the gap between where kV gets you and the actual target. It adds output proportional to the RPM error.

**Starting value:** `0.0001`

**Important: kP is in duty cycle per RPM of error, NOT volts.** Values are very small. A value of `0.001` at 1000 RPM error = 1.0 duty cycle (full power). Start small.

**Procedure:**
1. Keep kV at the value from step 1
2. Set `kP` to `0.0001`
3. Hold X to shoot and watch if the wheels now reach the target RPM
4. Check `AtSpeed?` on the dashboard — it should turn `true` when wheels are within tolerance
5. If the wheels still don't reach target, increase kP (try `0.0002`, `0.0005`)
6. If the wheels oscillate (RPM bounces up and down around target), decrease kP

**Signs of too much kP:**
- RPM oscillates around the target
- Motors make a buzzing/hunting sound
- Current spikes repeatedly

**Signs of too little kP:**
- Wheels don't quite reach target RPM
- `AtSpeed?` never turns true
- Takes a very long time to get close

#### 3. Tune kD (derivative) — optional

Most flywheel setups work fine with just kV + kP. Only add kD if you have oscillation that reducing kP doesn't fix.

**Starting value:** `0` (leave at zero unless needed)

**Procedure:**
1. Only if kP causes oscillation and reducing it makes the response too slow
2. Try a very small kD value (e.g., `0.00001`)
3. kD dampens rapid changes — it resists the oscillation
4. If it makes things sluggish, reduce kD

### Quick reference: what good tuning looks like

- Wheels spin up to target within ~0.5-1 second
- No oscillation — RPM is steady at target
- `AtSpeed?` is `true` and stays true
- Current draw is stable, not spiking
- When you release the button, wheels coast down smoothly

### Tuning the tolerance

`Launcher/Wheels/ToleranceRPM` (default 100) controls how close the wheels need to be to the target for `AtSpeed?` to read true. If your mechanism needs more precision, decrease it. If `AtSpeed?` flickers on and off, increase it.

### Live tuning workflow

1. Deploy to robot: `./gradlew deploy -Probot=LauncherTestbot`
2. Open Elastic/Shuffleboard, find Preferences
3. Change `Launcher/Wheels/kV`, `kP`, or `kD`
4. Release the button and press again — gains are pushed to the SparkMax at command start
5. Watch the dashboard values and repeat

### Troubleshooting

| Problem | Likely cause | Fix |
|---------|-------------|-----|
| Wheels don't spin at all | kV = 0, or wrong CAN ID | Check kV is nonzero, verify CAN IDs |
| Wheels go to 100% instantly | kP too high | Reduce kP (try 0.0001) |
| Wheels oscillate around target | kP too high | Reduce kP, or add small kD |
| Wheels reach ~80% but not target | kP too low or kV too low | Increase kP slightly |
| One wheel slower than the other | Mechanical friction, or different load | Tune is per-preset, not per-wheel. Check for mechanical issues. |
| `AtSpeed?` flickers | Tolerance too tight | Increase `Launcher/Wheels/ToleranceRPM` |
| Motor gets hot | Running too long at high current | Check current limit is set (40A default). Reduce RPM targets. |

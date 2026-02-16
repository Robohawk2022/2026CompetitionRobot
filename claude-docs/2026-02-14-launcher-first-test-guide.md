# Launcher First-Time Testing Guide

**Date**: 2026-02-14

This guide walks you through testing the launcher and agitator motors on the real robot for the first time. Follow every step in order.

---

## What You're Testing

You have **3 motors**, all REV NEO motors with SparkMax controllers:

| Motor | CAN ID | What it does |
|-------|--------|-------------|
| **Lower wheel** | 32 | Bottom launcher flywheel. Also spins backward for intake. |
| **Upper wheel** | 33 | Top launcher flywheel. |
| **Agitator** | 31 | Pushes balls toward (or away from) the launcher wheels. |

---

## What You Need

- Laptop with Driver Station and Elastic installed
- Xbox controller (or 8BitDo/Logitech - the code auto-detects)
- USB cable from laptop to roboRIO (or connect via WiFi)
- The robot on blocks or the launcher mechanism free to spin safely
- A second person ready to **disable the robot** from Driver Station if anything goes wrong

---

## Phase 1: Deploy the Code

### Step 1: Verify Main.java is correct

The file `src/main/java/frc/robot/Main.java` should have this active line:

```java
RobotBase.startRobot(LauncherTestbot::new);
```

All other `startRobot` lines should be commented out. This is already set correctly.

### Step 2: Build and deploy

Plug your laptop into the roboRIO via USB (or connect to the robot's WiFi).

```bash
./gradlew deploy
```

Wait for it to finish with `BUILD SUCCESSFUL`. If it fails, check your connection to the roboRIO.

---

## Phase 2: Connect and Set Up Driver Station

### Step 3: Open Driver Station

1. Open **FRC Driver Station** on your laptop
2. It should detect the roboRIO and show a green status on the Communications light
3. Plug your controller into the laptop via USB
4. In Driver Station, go to the **USB** tab (bottom-left) and verify your controller shows up on **port 0**
5. If the controller is on the wrong port, drag it to port 0

### Step 4: Verify the robot is DISABLED

The robot starts disabled. **Do not enable yet.** The big red DISABLE button in Driver Station should be active. Keep it that way until Phase 4.

---

## Phase 3: Set Up Elastic Dashboard

### Step 5: Open Elastic

1. Open **Elastic** on your laptop
2. It should auto-connect to the roboRIO's NetworkTables

### Step 6: Find the Preferences

In Elastic, look for the **NetworkTables** tree on the left side. Navigate to:

```
NetworkTables > Preferences
```

You should see all the settings listed below. **Verify these values before enabling the robot.** These are the safe defaults already in the code, but if anyone previously deployed code to this roboRIO, old values might be stored that override the defaults.

| Preference Key | Should Be | What It Controls |
|---|---|---|
| `Launcher/IntakeSpeedRPM` | **200** | How fast the lower wheel spins for intake/eject |
| `Launcher/Neutral/RPM` | **200** | How fast both wheels spin for shooting |
| `Launcher/HighArc/LowerRPM` | **200** | (Not used yet, but verify it's low) |
| `Launcher/HighArc/UpperRPM` | **200** | (Not used yet, but verify it's low) |
| `Launcher/Flat/LowerRPM` | **200** | (Not used yet, but verify it's low) |
| `Launcher/Flat/UpperRPM` | **200** | (Not used yet, but verify it's low) |
| `Agitator/ForwardPower%` | **10** | How hard the agitator pushes (10 = 10% power) |
| `Agitator/FeedPower%` | **10** | How hard the agitator feeds during a shot |
| `Launcher/LowerWheelInverted?` | **false** | Lower wheel direction |
| `Launcher/UpperWheelInverted?` | **false** | Upper wheel direction |
| `Agitator/Inverted?` | **false** | Agitator direction |

**If any RPM values are higher than 200, change them to 200 now.**

**If any power values are higher than 10, change them to 10 now.**

### Step 7: Add telemetry widgets to Elastic

Find these in the NetworkTables tree and drag them onto your Elastic dashboard so you can see them while testing:

**From LauncherSubsystem:**
- `Mode` - Shows which command is running (idle, intake, eject, spin-up)
- `LowerWheelCurrent` - Actual RPM the lower wheel is spinning
- `LowerWheelTarget` - RPM we're asking the lower wheel to reach
- `UpperWheelCurrent` - Actual RPM the upper wheel is spinning
- `UpperWheelTarget` - RPM we're asking the upper wheel to reach
- `LowerWheelAmps` - Current draw of lower motor (watch for danger!)
- `UpperWheelAmps` - Current draw of upper motor (watch for danger!)
- `AtSpeed?` - Whether both wheels have reached their target

**From AgitatorSubsystem:**
- `Mode` - Shows which command is running
- `RPM` - How fast the agitator motor is spinning
- `Amps` - Current draw of agitator motor

---

## Phase 4: Test Each Motor

### Controller Button Map

The controller is on port 0. Here are the buttons:

| Button | How to Use | What Happens |
|--------|-----------|-------------|
| **A** | **HOLD** it down | Intake - lower wheel spins backward, agitator pushes forward |
| **B** | **HOLD** it down | Eject - lower wheel spins forward, agitator pushes backward |
| **X** | **HOLD** it down | Shoot - both wheels spin forward, agitator feeds once at speed |
| **Y** | **TAP** it once | Emergency stop - kills all motors immediately |
| **Let go** | Release any button | Motors stop automatically (returns to idle) |

**"HOLD" means the motors only run while you hold the button. Let go and they stop.**

### SAFETY RULES

- **Have a second person sitting at Driver Station with their hand on the DISABLE button**
- **If ANYTHING sounds wrong - grinding, stalling, vibration - let go of the button immediately**
- **If letting go doesn't stop the motors, the second person clicks DISABLE in Driver Station**
- **Watch the Amps values in Elastic. If any motor stays above 30A for more than a second, let go**

---

### Test 1: Lower Wheel Direction (Intake)

This test runs ONLY the lower wheel and the agitator. Upper wheel stays off.

1. In Driver Station, select **Teleop** mode (not Auto, not Test)
2. Click **Enable**
3. **Tap A very briefly** (less than 1 second) and let go
4. Watch Elastic:
   - LauncherSubsystem `Mode` should flash to `intake` then back to `idle`
   - `LowerWheelTarget` should briefly show `-200` (negative = backward)
   - `UpperWheelTarget` should stay at `0`
5. If everything looks normal, **hold A for 2-3 seconds**
6. While holding A, **look at the physical lower wheel**:
   - It should be spinning in the direction that would **pull a ball INTO the robot**
   - The agitator should be pushing balls **toward the wheels**
7. **Let go of A**
8. All values should return to 0

**If the lower wheel is spinning the WRONG direction (pushing balls out instead of pulling in):**
- In Elastic Preferences, change `Launcher/LowerWheelInverted?` to `true`
- Press A again to test - the new direction takes effect immediately

**If the agitator is pushing the WRONG direction:**
- In Elastic Preferences, change `Agitator/Inverted?` to `true`
- Press A again to test

### Test 2: Lower Wheel Direction (Eject)

1. **Hold B for 2-3 seconds**
2. Watch Elastic:
   - Mode = `eject`
   - `LowerWheelTarget` = `200` (positive = forward)
   - Upper wheel stays off
3. **Look at the physical lower wheel** - it should spin the **OPPOSITE direction** from intake (pushing balls out)
4. The agitator should also reverse (pushing balls away from wheels)
5. **Let go of B**

### Test 3: Upper Wheel Direction (Shoot)

1. **Hold X for 2-3 seconds**
2. Watch Elastic:
   - Mode = `spin-up:NEUTRAL`
   - `LowerWheelTarget` = `200`
   - `UpperWheelTarget` = `200`
   - Both `Current` values should ramp up toward 200
3. **Look at BOTH physical wheels** - they should both spin in the direction that would **launch a ball OUT of the robot**
4. **Let go of X**

**If the upper wheel is spinning the WRONG direction:**
- In Elastic Preferences, change `Launcher/UpperWheelInverted?` to `true`
- Press X again to test

### Test 4: Emergency Stop

1. **Hold A** to start the intake
2. While motors are running, **tap Y**
3. Everything should stop immediately
4. Verify all values in Elastic go to 0

### Test 5: Disable Stop

1. **Hold A** to start the intake
2. While motors are running, have your partner click **DISABLE** in Driver Station
3. Everything should stop immediately

---

## Phase 5: Ramp Up Speed

Only do this AFTER all directions are confirmed correct in Phase 4.

**Always disable the robot before changing speeds. Then re-enable to test.**

### Round 1: 500 RPM

1. Click **DISABLE** in Driver Station
2. In Elastic Preferences, change:
   - `Launcher/IntakeSpeedRPM` = **500**
   - `Launcher/Neutral/RPM` = **500**
   - `Agitator/ForwardPower%` = **15**
   - `Agitator/FeedPower%` = **15**
3. Click **Enable**
4. **Hold A** for 3 seconds - intake test
   - Watch amps - should be well under 20A
   - Listen for smooth spinning, no grinding
5. Let go. **Hold B** for 3 seconds - eject test
6. Let go. **Hold X** for 3 seconds - shoot test
7. If everything sounds good, continue to Round 2

### Round 2: 1000 RPM

1. **DISABLE**
2. Change:
   - `Launcher/IntakeSpeedRPM` = **1000**
   - `Launcher/Neutral/RPM` = **1000**
   - `Agitator/ForwardPower%` = **20**
   - `Agitator/FeedPower%` = **20**
3. **Enable** and repeat the A / B / X tests
4. Watch amps more carefully - they will be higher now

### Round 3: 1500 RPM

1. **DISABLE**
2. Change:
   - `Launcher/IntakeSpeedRPM` = **1500**
   - `Launcher/Neutral/RPM` = **1500**
   - `Agitator/ForwardPower%` = **30**
   - `Agitator/FeedPower%` = **30**
3. **Enable** and repeat tests

### Round 4: 2000+ RPM (if needed)

Continue the same pattern, increasing by 500 RPM at a time. The theoretical max for a NEO motor is 5676 RPM, but you probably won't need to go above 3000-4000 RPM for competition.

For the agitator, increase by 10% per round. You probably won't need more than 50%.

---

## What to Watch For (Danger Signs)

| What you see | What it means | What to do |
|---|---|---|
| Amps above 30 and climbing | Motor is stalling against something | **Let go immediately** |
| Motor not spinning but amps > 0 | Motor is stalled / jammed | **DISABLE immediately** |
| Grinding or clicking noise | Mechanical problem | **DISABLE and inspect** |
| `LowerWheelCurrent` not tracking `LowerWheelTarget` | PID might need tuning, or motor isn't connected | OK at low RPM, investigate if it persists at higher RPM |
| Burning smell | Motor or wiring overheating | **DISABLE immediately, do not re-enable** |
| RPM overshoots wildly past target | PID gains too aggressive | Reduce `kP` value in Preferences |

---

## Quick Reference: All Preference Keys

### Speeds (change these to ramp up)

| Key | Default | What it does |
|---|---|---|
| `Launcher/IntakeSpeedRPM` | 200 | Lower wheel RPM for intake (A button) and eject (B button) |
| `Launcher/Neutral/RPM` | 200 | Both wheels RPM for shooting (X button) |
| `Agitator/ForwardPower%` | 10 | Agitator power for intake/eject (% of max voltage) |
| `Agitator/FeedPower%` | 10 | Agitator power for feeding a shot (% of max voltage) |

### Motor Direction (flip if motor spins wrong way)

| Key | Default | What it does |
|---|---|---|
| `Launcher/LowerWheelInverted?` | false | Flip lower wheel direction |
| `Launcher/UpperWheelInverted?` | false | Flip upper wheel direction |
| `Agitator/Inverted?` | false | Flip agitator direction |

All of these update live - change them in Elastic, then press a button on the controller and the new value takes effect immediately.

### PID Tuning (don't change these until speeds are working)

| Key | Default | What it does |
|---|---|---|
| `Launcher/Lower/kV` | 0.002 | Lower wheel feedforward (volts per RPM) |
| `Launcher/Lower/kP` | 0.0001 | Lower wheel proportional gain |
| `Launcher/Upper/kV` | 0.002 | Upper wheel feedforward (volts per RPM) |
| `Launcher/Upper/kP` | 0.0001 | Upper wheel proportional gain |

### Safety Limits (don't change these)

| Key | Default | What it does |
|---|---|---|
| `Launcher/CurrentLimit` | 40 | Max amps per launcher wheel motor |
| `Agitator/CurrentLimit` | 40 | Max amps for agitator motor |

---

## After Testing: What to Write Down

Write down and tell your programming team:

1. Which motors needed inversion? (Lower wheel, upper wheel, agitator)
2. What RPM did you reach successfully?
3. Did any motor sound or behave strangely?
4. What amps did each motor draw at your final RPM?
5. Did the `Current` RPM track the `Target` RPM well, or was it way off?

# Oct 24 Prep: Robot Checklist (do at next practice)

**Written**: 2026-09-21
**Related**: `2026-09-21-brownout-branch-review.md` (what changed and why)

## Where things stand

- The brownout changes (current limits, steer encoder setting, rotation fix, logging) are
  on the `bethesda-auto` branch **on Michelle's laptop only, NOT committed**. Deploy from
  that laptop, or the robot won't get them.
- Nothing has been tested on the robot yet. Everything below still needs doing.
- When you come back to Claude, say "continue the Oct 24 checklist" and point it at this
  file. Bring the PDH list from step 3 and any notes from steps 2 and 4.

## Progress

- [ ] 0. Deploy the new code
- [ ] 1. USB stick in the roboRIO
- [ ] 2. Tuner X: limits correct, unlicensed-feature fault gone
- [ ] 3. PDH slot list written down (give to Claude to put in the code)
- [ ] 4. Teleop drive check
- [ ] 4. Auto check (at least SLAY)

---

## 0. Before you start: get the new code onto the robot

1. Fix Java on the laptop. `JAVA_HOME` points at a folder that doesn't exist. In a
   terminal, run:
   ```
   export JAVA_HOME=$HOME/Library/Java/JavaVirtualMachines/corretto-21.0.11/Contents/Home
   ```
   This only lasts for that terminal window. (Deploying from WPILib VS Code with
   **"WPILib: Deploy Robot Code"** also works.)
2. Check you're on the right branch: `git status` should say `On branch bethesda-auto`
   and list the modified files.
3. Turn the robot on and connect the laptop to it (Wi-Fi to the robot radio, or a
   USB/Ethernet cable).
4. From the project folder, run `./gradlew deploy`. Wait for **BUILD SUCCESSFUL**.

---

## 1. Put a USB stick in the roboRIO for the logs

**Why:** the new code records battery voltage, motor currents and brownouts to files.
Without a stick they go on the roboRIO's small internal storage, which can fill up.

1. **Get a USB stick**, 8–32 GB. It will be erased.
2. **Format it on the Mac:**
   - Open **Disk Utility** (Cmd+Space, type "Disk Utility").
   - Click the stick in the left sidebar. Choose the stick itself, not the indented
     volume under it. If you can't see it, pick **View → Show All Devices**.
   - Click **Erase**, choose Format **MS-DOS (FAT)** and Scheme **Master Boot Record**,
     then click **Erase**.
   - Eject it.
3. **With the robot powered off**, plug the stick into a USB-A port on the roboRIO (the
   rectangular ports, same as on a laptop).
4. **Power on the robot.** The code checks for a stick when it starts, so if you plug it
   in while the robot is on, restart the robot code from the Driver Station.
5. **Check that it's working:** enable for a minute, disable, power off, and plug the
   stick into the laptop. You should see `.wpilog` and `.hoot` files, usually in a `logs`
   folder. If they're there, put the stick back in the robot.
6. **At the event:** leave it in all day. Afterwards, the files get read: `.wpilog` in
   AdvantageScope (comes with WPILib), `.hoot` in Tuner X.

---

## 2. Check the new current limits in Tuner X

**Why:** this confirms the new motor settings actually reached the motors.

**Motor IDs:**

| Module | Drive motor | Steer motor |
|---|---|---|
| Front left | 41 | 42 |
| Front right | 21 | 22 |
| Back left | 11 | 12 |
| Back right | 31 | 32 |

1. Robot on, **new code deployed**, laptop connected. Leave the robot disabled.
2. Open **Phoenix Tuner X** (CTRE's app; it runs on Windows, so probably on the driver
   station laptop).
3. Connect to the robot: at the top, enter team **3373** (or the address `10.33.73.2`)
   and connect. A list of devices should appear.
4. **Check one drive motor:** click **Talon FX 41**, open the **Configs** tab, type
   `current` in the search box. Expected:
   - Stator Current Limit **80**, Stator Current Limit Enable **✓**
   - Supply Current Limit **60**, Supply Current Limit Enable **✓**
   - Supply Current Lower Limit **40**
   - Supply Current Lower Time **0.5**
5. **Check one steer motor:** click **Talon FX 42**, Configs, search `current`:
   - Stator Current Limit **60**, enabled
   - Supply Current Limit **40**, enabled
   - Supply Current Lower Limit **25**
   - Supply Current Lower Time **0.5**
6. If 41 and 42 look right, spot-check one more pair (say 21 and 22).
7. **Check the "unlicensed feature" fault**, which should be gone now:
   - Click **Talon FX 42** and look at its faults (the fault/"Self Test Snapshot" area).
   - Click **Clear Sticky Faults**. Old faults stay listed until cleared.
   - Restart the robot code from the Driver Station (or power-cycle), wait 30 seconds,
     and look again.
   - If **UnlicensedFeatureInUse** comes back, write it down for Claude.

**If any number is wrong:** don't change it in Tuner X, because the code overwrites it
every time it starts. Take a photo of the values.

Result notes:
```


```

---

## 3. Work out what's on each PDH breaker slot

**Why:** the logs record current for each of the PDH's 24 slots. Right now they're
labeled "Ch0", "Ch1", and so on. With names, the logs will say "FL Drive pulled 60 A"
instead of "Ch7 pulled 60 A".

The **PDH** (Power Distribution Hub) is the box the battery's main red and black cables
go into, with a row of snap-in breakers.

1. **Power off the robot.**
2. Each breaker slot has a **number printed next to it (0–23)**.
3. For each breaker with something plugged in, **follow its red/black wire pair** to
   where it ends: a motor (Krakens, NEOs), a motor controller (SPARK MAXes), or another
   device (radio, Limelight, and so on).
4. Fill in the list below. If you can't tell which swerve motor is which, most Krakens
   have their CAN ID on a sticker (see the table in step 2); "drive motor, front-left
   corner" is fine too.
5. Give the list to Claude to put in `PDH_CHANNEL_NAMES` in `RobotContainer.java`.
   (There's a limit on how many names that code format allows, so let Claude do it.)

PDH list:
```
0  =
1  =
2  =
3  =
4  =
5  =
6  =
7  =
8  =
9  =
10 =
11 =
12 =
13 =
14 =
15 =
16 =
17 =
18 =
19 =
20 =
21 =
22 =
23 =
```

---

## 4. Drive the robot and run the autos

**Why:** the drive motors are now limited to 80 A instead of 120 A, so launches may feel a
little softer, and the steering sensor setting changed. Confirm nothing is off before the
event, not at it.

**Setup:** a fully charged battery, open floor space, someone on the Driver Station with a
finger on disable (**spacebar** disables immediately).

### Teleop check (about 5 minutes)

1. Enable in **Teleop**.
2. Drive forward, back, left and right, and spin in place both ways. **Watch the wheels:**
   they should point cleanly in the direction you drive, with no wobbling or wheels
   pointing the wrong way. If they wobble or fight each other, disable right away and
   note it. That would point to the steering sensor change.
3. Do a few hard starts from standstill. Slightly less punchy than before is expected.
   Note it if it feels *much* weaker.
4. Hold the **left bumper** (sniper mode) and rotate. It should turn slower. Release it
   and rotation should go back to normal.
5. Try intake (left trigger), shoot (right trigger) and jiggle (X) to make sure nothing
   else changed.

### Auto check

**Choosing an auto:** the Digit Board (the little 4-character display) shows the selected
program. Press **B** to go forward through the list and **A** to go back:

| Display | Auto |
|---|---|
| CHUD / UNC | do nothing (skip these) |
| 67L / 67R | shoot only |
| BRUH / RIZZ | depot routines |
| **SLAY** | **the Bethesda auto (most important)** |

For each auto you might use on Oct 24, at minimum **SLAY**:

1. Put the robot in that auto's starting spot, the same place used at Bethesda.
2. Pick it on the Digit Board.
3. On the Driver Station, choose **Autonomous** mode, then **Enable**.
4. Watch it run. Same route, ends in about the same place, scores like it did at
   Bethesda?
5. Disable, reset the robot and balls, and go on to the next auto.

**If an auto looks different:** write down which auto and what changed (for example,
"SLAY stopped short of the depot" or "turned late"). A difference isn't expected, since
every path caps itself well below the new limits, but if there is one it can be adjusted.

Result notes:
```


```

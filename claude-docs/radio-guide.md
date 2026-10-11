# Radio Setup Guide (VH-109)

Last updated 2026-10-03. Source:
[WPILib: Programming your Radio](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html).
Check that page if anything below doesn't match what you see, since firmware and steps change between seasons.

This is for the **Vivid-Hosting VH-109**, the standard FRC radio since 2024. The older
OpenMesh radio uses a different process (the FRC Radio Configuration Utility).

## At an event: use the kiosk

Don't program the radio yourself at a competition. Take it to the **radio kiosk**, and they
set it up for the field. Everything below is for home and practice.

## 1. Connect to the radio

1. Power the radio, either from the robot or with PoE (power over the Ethernet cable).
2. Run an Ethernet cable from your laptop to the port labeled **DS** on the radio.
3. In a browser, go to **http://radio.local/**.
4. If that page doesn't load, set the laptop's Ethernet to a static IP of **192.168.69.2**
   with subnet **255.255.255.0**. Then go to **http://192.168.69.1/**.

## 2. Update the firmware (if needed)

1. Download the firmware from Vivid-Hosting's firmware releases page. Pick the **Radio** version.
2. Copy the SHA-256 checksum listed next to the download.
3. On the radio's page, under Firmware Upload, paste the checksum into the **Checksum** box.
4. Click **Browse…**, choose the file, then click **Upload**.
5. **Don't unplug it** for 2–3 minutes. It's done when the PWR light is solid and the SYS
   light blinks slowly, about once a second.

## 3. Set it up as the robot radio

1. Select **Robot Radio Mode**.
2. Enter the team number, **3373**.
3. Optionally, add a suffix to tell your robots apart.
4. Enter the **6 GHz WPA/SAE key**. It has to match the access point radio.
5. Enter the **2.4 GHz WPA/SAE key**. This is the Wi-Fi password laptops use to connect.

6. Check **Enable 2.4 GHz Wi-Fi** if you want laptops to connect to the radio directly
   (no access point radio).
7. Click **Configure**. "Success, new configuration will be applied asynchronously" means it
   worked; wait about 2 minutes while the radio applies it.

Don't write the actual keys in this file or anywhere else in the repo.

### Connecting a laptop over Wi-Fi without an access point radio

The radio only creates its own Wi-Fi network when **all** of these are true:

- **DIP switch #3** on the bottom of the radio is **ON**. Power the radio off before flipping it.
- **Enable 2.4 GHz Wi-Fi** is checked on the config page.
- The radio is **not** connected to a 6 GHz access point radio. Once it connects to one, the 2.4 GHz
  network shuts off until the radio is power cycled.

The network name is **`FRC-3373`**, not `3373`. With a suffix it's `FRC-3373-<suffix>`. The password is
the 2.4 GHz key. Source: [Vivid-Hosting: Programming Your Radio At Home](https://frc-radio.vivid-hosting.net/overview/programming-your-radio-at-home).

A healthy, running radio has a solid **PWR** light and a **SYS** light blinking slowly (about once a second).

## 4. Set up the access point radio (home Wi-Fi, if you use one)

Use a second VH-109 and repeat step 3, but choose **Access Point Mode**. The team number,
suffix and both keys must be exactly the same as on the robot radio.

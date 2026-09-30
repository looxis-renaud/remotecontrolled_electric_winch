# VESC Tool guide for this winch

How to connect to the winch's VESC, read faults, use the realtime data, change settings and redo the hall sensor setup. Written so it can be followed again months later. Everything refers to the current configuration of this winch:

- Motor config: [260930_motor_config.xml](260930_motor_config.xml)
- App config: [260930_app_config.xml](260930_app_config.xml)

Background (what the settings mean, why hall mode, autostop): [readme.md](readme.md). Motor data and sensor wiring: [doc/qs-motor/README.md](../doc/qs-motor/README.md).

> ⚠ **Safety first.** Whenever the motor can turn (detection, potentiometer, test buttons in VESC Tool): winch secured, **line without load, nobody at the drum**. Don't brake from high speed until [WINCH-22](../features/WINCH-22-overvoltage-regen-braking.md) (over-voltage when braking) is resolved.

## 1. Versions: don't update

| Part | Version | Note |
|------|---------|------|
| VESC firmware | **5.3** with the autostop patch | Confirmed 2026-09-30. Hardware `75_300_R3`. |
| VESC Tool (desktop) | **3.01** | Matches FW 5.3. |
| VESC Tool (smartphone) | *to be completed* | See section 11. |

- **Never accept a firmware update** offered by VESC Tool. It would replace the patched firmware with a stock firmware **without line autostop**.
- A newer VESC Tool may not work properly with FW 5.3 or may push you to update the firmware. Stay on 3.01 for this winch.

## 2. Connecting

1. Winch battery on, VESC on (power switch), LED blue, then dim green = firmware running.
2. USB cable (Mini-USB on this controller) to the laptop.
3. VESC Tool → **Connection** (left menu) → select the serial port (e.g. COM3) → **Connect** (or "Autoconnect" on the welcome page).
4. Bottom right should show "Connected (serial) to COM…". Left, under **CAN-Devices**: `VESC 75 300 R3 local`.
5. **Firmware** page (left menu) shows firmware and hardware version: `Fw: 5.3, Hw: 75_300_R3`. Check it once per session.

## 3. Reading and writing settings (the most important rule)

On the right edge of VESC Tool there is a column of buttons for the **motor** configuration (M) and the **app** configuration (A):

| Button (go by the tooltip) | What it does |
|----------------------------|--------------|
| **Read** configuration | Loads the settings **from the VESC** into VESC Tool. |
| **Read default** configuration | Loads **factory defaults** into VESC Tool. ⚠ Never write these to the winch. |
| **Write** configuration | Sends the settings in VESC Tool **to the VESC**. In VESC Tool 3.01 this is the icon with the arrow pointing **down**. |

Rules:
1. **Always read first**, then change, then write. VESC Tool does not load the VESC's settings by itself; without reading you edit (and write) whatever happens to be in VESC Tool.
2. A change in VESC Tool does **nothing** until it is written.
3. Motor settings and app settings are written separately (M and A).
4. After writing: read again and check the value really changed.

### Backups (XML)
- Before and after every change: **read**, then **File → Save Motor Configuration XML as…** and **File → Save App Configuration XML as…**
- Name: date prefix, e.g. `260930_motor_config.xml`. Use a new name so the "before" state is not overwritten.
- Commit the new backups to `vesc/`, move superseded ones to `old/vesc-configs/`.
- **Never load an old or foreign XML and write it to the winch** (it contains motor measurements, hall table and limits of another state or motor).

## 4. Terminal: reading faults

Left menu → **VESC Dev Tools** (or the Terminal menu at the top) → **Terminal**. Type a command, press Enter.

| Command | What it shows |
|---------|---------------|
| `faults` | All faults since the VESC was switched on, with a snapshot of the values at that moment. "No faults registered" if there were none. The list is cleared by a restart. |
| `help` | List of all terminal commands of this firmware. |

Example from 2026-09-30 (sensor cable connected, wrong temperature sensor type):

```
Fault            : FAULT_CODE_OVER_TEMP_MOTOR
Current          : -0.7         motor current at the moment of the fault (A)
Voltage          : 63.45        battery voltage (V)
Duty             : -0.000
RPM              : 0.1          electrical RPM (ERPM), see section 7
Tacho            : 4            tachometer counts, see section 8
Temperature      : 27.66        MOSFET (controller) temperature, not motor temperature
```

### Fault codes that matter on the winch

| Fault | Meaning | What to check on this winch |
|-------|---------|-----------------------------|
| `FAULT_CODE_OVER_TEMP_MOTOR` | Motor temperature above `l_temp_motor_end` (85 °C) → no current. | Motor really hot? Otherwise: **sensor cable unplugged** or **wrong sensor type** (must be KTY83/122, section 6). Happened 2026-09-30 with the NTC setting. |
| `FAULT_CODE_OVER_TEMP_FET` | Controller above 100 °C. | Cooling fan running? Airflow in the enclosure? |
| `FAULT_CODE_OVER_VOLTAGE` | Battery voltage above `l_max_vin` (72 V). | Happens when **braking** feeds energy back and the battery doesn't take it. Happened 2026-09-30 when braking from full speed (fault at 72.1 V) → [WINCH-22](../features/WINCH-22-overvoltage-regen-braking.md). |
| `FAULT_CODE_UNDER_VOLTAGE` | Battery voltage below `l_min_vin` (12 V). | Battery empty, BMS switched off, loose connector. |
| `FAULT_CODE_ABS_OVER_CURRENT` | Current above `l_abs_current_max` (290 A). | Very hard pull/brake, wrong motor measurement, wiring fault. |
| `FAULT_CODE_DRV` | Gate driver fault (power stage). | Serious: stop, check phase wires, contact Trampa. |

The **red LED** on the VESC means a fault is active. Always read `faults` before the next tow.

## 5. Realtime data

Left menu → **Data Analysis → Realtime Data**. Start the stream with the **"Stream realtime data"** button (right edge, "RT"). Tabs: Current, Temperature, RPM, FOC, Rotor Position, Experiment.

The **Current** tab plots three lines:

| Line | Meaning |
|------|---------|
| Current in (battery current, I Batt) | Current from the battery (positive) or back into it (negative = braking with energy recovery). |
| Current motor (I Motor) | Current in the motor windings. **This is the pull (torque):** ≈ 3.4–3.6 A per kg of line pull. Negative = braking. |
| Duty cycle | How much of the battery voltage the VESC puts on the motor (0–95 %). Roughly proportional to speed. At 95 % the motor can't go faster. |

At low speed the motor current can be much higher than the battery current (the VESC works like a gearbox for current). That is normal.

Values below the plot:

| Field | Meaning | Normal on the winch |
|-------|---------|---------------------|
| Power | Battery power (W), negative when braking | |
| Duty | Duty cycle in % | 0 at standstill |
| ERPM | Electrical RPM (section 7) | 0 at standstill |
| I Batt / I Motor | See above | |
| T FET | Controller temperature | Ambient to ~60 °C; limits 85/100 °C |
| T Motor | Motor temperature (KTY83 sensor) | Cold motor ≈ ambient. Limits 75/85 °C |
| Fault | Current fault, `NONE` if OK | `NONE` |
| Tac / Tac ABS | Tachometer counts (section 8). Tac counts up and down, Tac ABS only up | |
| Ah / Wh Draw / Charge | Energy drawn / recovered since power-on | |
| Volts In | Battery voltage | 48–67.2 V. **Above 72 V = over-voltage fault** |

**Temperature** tab: T FET and T Motor over time. Useful for the heating test of the motor sensor (warm the motor with a hair dryer, T Motor must rise).

**Rotor Position** tab: with hall mode, you can turn the drum slowly by hand and watch the position change smoothly (no jumps). Useful after the hall detection.

### ⚠ The control bar at the bottom
The bottom bar has fields and play buttons for **D** (duty), **ω** (speed), **IB / I** (current), **P** (position), **HB** (handbrake) and a red **STOP** button. The play buttons **drive the motor directly** from the laptop, bypassing the remote. Don't use them on the winch unless the winch is secured and nobody is near the drum. **STOP** releases the motor.

## 6. Changing the motor temperature sensor type

This winch's motor has a **KTY83-122** temperature sensor (0.97 kΩ at ~25 °C, see [doc/qs-motor/README.md](../doc/qs-motor/README.md)).

1. **Read** motor configuration.
2. **Motor Settings → General → tab "Advanced" → Motor Temperature Sensor Type = KTY83/122.** (Not in the "Temperature" tab; that one only has the limits.)
3. Leave the limits in the Temperature tab unchanged: Motor Temp Cutoff Start **75 °C**, End **85 °C**.
4. **Write** motor configuration, then check T Motor in Realtime Data (≈ ambient when cold).

In [260930_motor_config.xml](260930_motor_config.xml): `m_motor_temp_sens_type = 2` (KTY83/122). Until 2026-09-30 it was `0` (NTC 10k): the KTY83's ~1 kΩ then reads as ~100 °C → `FAULT_CODE_OVER_TEMP_MOTOR`, red LED, no current.

With KTY83/122, an **unplugged sensor cable reads as extremely hot** → same fault. The sensor cable must always be connected.

Also in the Advanced tab: **Auxiliary Output Mode** = "Temp motor or mosfet > 50 C" (`m_out_aux_mode = 11`): the VESC's switched AUX output turns on above 50 °C.

## 7. ERPM, speed ranges, hall sensors and HFI

### What ERPM is
VESC Tool shows speed as **ERPM** (electrical revolutions per minute). The motor has 32 magnets = **16 pole pairs**, so one turn of the drum is 16 electrical revolutions:

- **drum rpm = ERPM ÷ 16**
- **line speed (m/s) = drum rpm × drum circumference ÷ 60**. Empty drum: diameter 0.43 m (`si_wheel_diameter`), circumference π × 0.43 ≈ 1.35 m. With line wound on, the effective diameter is larger, so the real line speed is somewhat higher.

| ERPM | Drum rpm | Line speed (empty drum) | Meaning in the config |
|-----:|---------:|------------------------|-----------------------|
| 150 | 9 | 0.2 m/s (0.8 km/h) | `foc_openloop_rpm` (only used in sensorless mode) |
| 500 | 31 | 0.7 m/s (2.5 km/h) | `foc_hall_interp_erpm`: hall interpolation starts |
| 2000 | 125 | 2.8 m/s (10 km/h) | `foc_sl_erpm`: switch from hall sensors to sensorless observer |
| 4300 | 269 | 6.1 m/s (22 km/h) | example from the realtime data on 2026-09-30 |
| ≈ 12 600 | ≈ 790 | ≈ 18 m/s (64 km/h) | rough theoretical maximum at 63 V and 95 % duty without load (calculated from the flux linkage, not measured) |

### How the VESC knows the rotor position in hall mode (`foc_sensor_mode = 2`)

| Speed | Position from | How |
|-------|---------------|-----|
| **0 – 500 ERPM** (0 – 0.7 m/s) | **Hall sensors** | The 3 hall sensors give 6 states per electrical revolution = 96 steps per drum turn (every 3.75°, about every 14 mm of line). The VESC uses the measured **hall table** to know which rotor angle each state means. Works at standstill. |
| **500 – 2000 ERPM** (0.7 – 2.8 m/s) | **Hall sensors, interpolated** | Between two hall edges the VESC estimates the angle from the current speed, so the position is smooth instead of in 3.75° steps. |
| **above 2000 ERPM** (above 2.8 m/s) | **Sensorless observer** | The VESC calculates the position from the back-EMF (the voltage the turning motor generates), using R, L and flux linkage from the motor measurement. More precise at speed. |

This is why hall mode matters on the winch: pre-tension with the pilot standing, brake ↔ pull transitions, braking to standstill and autostop all happen **below 2.8 m/s**, where the sensorless observer is weak and the hall sensors are reliable.

### And HFI?
**HFI (high frequency injection) is not used on this winch.** HFI is a separate sensor mode (`foc_sensor_mode` = HFI and its variants) for motors **without** sensors: the VESC injects an audible high-frequency signal to find the rotor position at standstill. It needs careful tuning. The `foc_hfi_*` values in the config are just defaults and have no effect in hall mode.

For comparison, **sensorless mode** (`foc_sensor_mode = 0`, used until 2026-09-30): below the observer's working range the VESC starts "blind" in open loop up to `foc_openloop_rpm` (150 ERPM) and hopes the rotor follows. Under load this can jitter or twitch backwards, which is exactly what hall mode avoids.

### Where to see and set it
- **Motor Settings → FOC → General → Sensor Mode**: `Hall Sensors` (not Sensorless, not HFI).
- **Motor Settings → FOC → Hall Sensors** tab: Sensorless ERPM (2000), Hall Interpolation ERPM (500), Hall Table [0]…[7].
- **Motor Settings → General → Sensor Port Mode: Hall Sensors** only says what kind of sensor could be on the port. It does not switch hall mode on.

## 8. Tachometer and line length

The tachometer counts the motor's position steps. As far as known for FW 5.x: **6 counts per electrical revolution = 96 counts per drum turn** (to be verified, see below). Tac counts up and down with direction; Tac ABS only counts up. Both start at 0 when the VESC is switched on (with the line fully wound in).

With the empty drum (circumference ≈ 1.35 m) that is about **71 counts per metre of line**, fewer with line wound on (larger effective diameter).

> ⚠ **Calculated, to be verified:** the autostop patch assumes `1500 counts ≈ 15 m`, and `receiver.ino` assumes 100 counts per metre (it sends `tachometer / 1000` as tens of metres). That fits a drum circumference of about 0.97 m (Robert Zach's winch, 0.31 m drum). On this winch (0.43 m drum) it would mean:
> - the remote shows about **0.71× the real line length**, which matches the observed "about 0.7×" (see CLAUDE.md, known quirks);
> - the VESC autostop triggers at 1500 counts ≈ **21 m** of line left, not 15 m.
>
> Check: pull out a measured length (e.g. 10 m) and read Tac in Realtime Data. Tracked in [WINCH-08](../features/WINCH-08-autostop-docs-verification.md).

## 9. Hall sensor detection (redo only when needed)

Needed again only after changing the motor, the sensor cable or its wiring. Full background: [readme.md](readme.md), "Hall sensors: how they were set up".

1. Backup (section 3).
2. Potentiometer at the "off" end. Winch secured, **line without load, nobody at the drum: the motor turns by itself.**
3. **Motor Settings → FOC → Hall Sensors** tab → start the hall detection (default detection current; increase in small steps if the motor doesn't turn).
4. Check the table: entries **1–6 must have values, 0 and 7 = 255**. Otherwise a sensor or wire is faulty: don't use hall mode.
5. Apply the table, **FOC → General → Sensor Mode = Hall Sensors**, **write** motor configuration.
6. Test with a little potentiometer travel: smooth start and smooth braking at low speed. Then `faults`.
7. Read, save XML, commit.

**Don't use the "Setup Motor FOC" wizard for this.** It also resets current limits, battery and temperature settings.

Result on 2026-09-30: hall table `255, 65, 118, 103, 199, 34, 173, 255` (`foc_hall_table__0` … `__7` in [260930_motor_config.xml](260930_motor_config.xml)).

## 10. Things not to do on the winch

- Firmware update (section 1).
- **Read default** + write.
- Load an old or foreign XML and write it.
- **Setup Motor FOC** wizard without a plan and a backup (resets limits).
- **Setup Input** wizard: it changes the PPM pulse range = how many amps each pull state means (safety-relevant, [WINCH-14](../features/WINCH-14-pull-value-calibration.md)).
- Drive the motor from the control bar at the bottom without securing the winch.
- Brake from high speed (until [WINCH-22](../features/WINCH-22-overvoltage-regen-braking.md) is resolved).

## 11. Smartphone app

*To be completed.* Etienne has the VESC Tool smartphone app. Open points:
- Which version, and does it work with FW 5.3? (Newer app versions may not support FW 5.3 fully or may offer a firmware update: decline it.)
- How does it connect to this VESC (Bluetooth module on the VESC? USB-OTG on Android?)
- Which of the functions above are available in the app (faults, realtime data, reading settings)?

## 12. Worked example: 2026-09-30

1. Red LED with the sensor cable connected → `faults` → `FAULT_CODE_OVER_TEMP_MOTOR`.
2. Backups saved. The XML showed `m_motor_temp_sens_type = 0` (NTC 10k), but the motor has a KTY83-122.
3. Sensor type changed to KTY83/122 (section 6), written → LED off after a restart, potentiometer works.
4. Full speed with the potentiometer, then fast braking → `FAULT_CODE_OVER_VOLTAGE` (72.1 V, up to 88 V shown) → [WINCH-22](../features/WINCH-22-overvoltage-regen-braking.md).
5. Hall detection (section 9), sensor mode Hall, written → smooth start and braking at low speed.
6. New backups committed as `260930_*.xml`, old ones moved to `old/vesc-configs/`.

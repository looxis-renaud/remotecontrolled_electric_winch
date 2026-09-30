# VESC Introduction
VESC is the Open Source Electronic Speed Controler developed by Benjamin Vedder ( **V**edder **E**lectronic **S**peed **C**ontroller).

# The controller used on this winch: Trampa VESC 75/300

The winch runs on a **Trampa VESC 75/300** (hardware revision `75_300_R3`), driving the QS Motor 12kW 260 V4 hub motor in FOC mode.
It is flashed with the patched autostop firmware [vesc_75_300_auto_stop.bin](vesc_75_300_auto_stop.bin) (see "Line auto stop in VESC" below).

Manufacturer manual (PDF): [VESC-75-300-MKIV-MANUAL.pdf](VESC-75-300-MKIV-MANUAL.pdf)

Product page: https://trampaboards.com/vesc-75v-300a-black-anodised-non-conductive-cnc-housing--the-most-powerful-vedder-electronic-speed-controller-ever-p-26284.html

## What matters for the winch

| Topic | Controller spec | What it means for this winch |
|-------|-----------------|------------------------------|
| Voltage | 12V – 67.2V (4S–16S LiPo/Li-ion), spikes must never exceed **75V** | Our 16S battery is at exactly **67.2V** when fully charged, which is the upper limit. Braking (brake states, autostop) is regenerative and feeds energy back into the battery, raising the voltage. Braking hard with a freshly, fully charged battery is therefore the most critical case for overvoltage. Never charge above 4.2V/cell. |
| Current | 300A continuous, 400A burst, **depending on mounting, ambient temperature and air circulation** | The continuous rating assumes good cooling. That is why the VESC has a cooling fan (see main README), and why the enclosure needs airflow around the heat sink. |
| Regenerative braking | Energy is recovered during braking | Brake states (-1 soft / -2 hard) and autostop brake with motor current; see voltage note above. |
| Protection | Under/over voltage, over current, over temperature (motor and ESC) | If a limit is hit, the VESC reduces or cuts motor power, including **during a tow**. Keep the battery charged and the controller cool. |
| Real-time data | Motor temperature, current, voltage | The receiver reads battery voltage, motor temperature, tachometer and duty cycle via UART and shows them on the remote. |
| Aux power outputs | 12V 1A (switchable), 5V 1A, 3.3V 0.5A; **all combined max. 1A** | Everything powered from these outputs (e.g. cooling fan, relay, potentiometer, receiver, depending on the build) shares this 1A budget. |

## From the manual: wiring & safety notes

Winch-relevant points from the manufacturer manual ([PDF](VESC-75-300-MKIV-MANUAL.pdf)). The PDF is the authoritative source; this is only a summary.

> **Note:** The manual describes a **newer revision (MKVI)** than the controller on this winch. Connector types and positions differ: the winch's controller has **Mini-USB** instead of USB-C, and its power-switch connector sits in a different place. The general wiring and safety rules still apply, but check port positions and pinouts on the actual device, not in the manual's drawings.

**Manufacturer warning.** Trampa states the VESC "may not be used for applications requiring fulfillment of special safety standards", explicitly naming aircraft and safety-critical environments. A tow winch is exactly such an application. The winch uses the controller anyway, so the other safety layers (VESC autostop, receiver failsafe, the rewind rule in the main README) must never be weakened or skipped.

**Minimum power path (battery → VESC):**
- Safety power cut-off.
- Fuse rated for the weakest part of the electrical system.
- Anti-spark / pre-charge: anti-spark connectors (e.g. XT90S) and/or an anti-spark switch. **Never power the VESC without pre-charge**; the capacitors must charge slowly, otherwise permanent damage may result.
- A BMS is required when the motor is used for regenerative braking. On the winch this is always the case (brake states and autostop).
- All 3 battery input cables and all 3 motor phase wires must be connected and properly insulated; wire gauge must match the current.
- Keep the controller dry; the enclosure must protect it against water.

**Connectors** (as described for the MKVI: signal ports JST-PH, 2 mm pitch. On the winch's older controller USB is Mini-USB and connector positions differ):

| Port | Manual | On this winch |
|------|--------|---------------|
| PPM | Input from an RC receiver. Never connect one receiver to several VESCs (Y-PPM); use opto decouplers. | PPM signal from the LoRa receiver (IO13). |
| COMM | UART, I²C and ADC; on the MKVI also the power-switch pin. | UART telemetry to the receiver; potentiometer on ADC2 (see "Line auto stop in VESC"). |
| Sensors | Hall, ABI or AS5047P motor position sensors (3.3 V or 5 V). | *TODO (Etienne): confirm whether the motor's hall sensors are connected.* |
| Motor A/B/C | Phase colours: A = yellow, B = blue, C = red (for correct display in VESC Tool). | QS Motor phases. |
| CAN | Only connect CAN H and CAN L; all devices on the same battery GND. | Not used. |
| USB | Configuration, firmware update, real-time data (USB-C on the MKVI). | VESC Tool via **Mini-USB**. |
| AUX | 12 V out (SIG, software-switched). | Counts towards the 1 A aux budget above. |

**LED codes** (useful for troubleshooting):

| LED | Meaning |
|-----|---------|
| Blue | Powered up |
| Green, dim | Firmware running |
| Green, bright | Driving the motor |
| **Red** | **Fault.** Connect VESC Tool and read out the fault code before the next tow. |

**Power switch.** For the MKVI, the manual describes a momentary **normally closed (NC)** switch on the COMM power-switch pin, with three wiring options (illuminated NC switch, plain NC switch, or a separate soft-start switch without auto power-off). This does not apply 1:1 to the winch's older controller, whose power-switch connector is in a different place.

**On this winch:** an on/off switch on the controller's power-switch connector. **Switch open = VESC off, switch closed = VESC on.**

*Source: Trampa VESC 75/300 manual, pages 1, 3 and 4.*

## Technical specifications (manufacturer data)

**Voltage**
- 12V – 67.2V (safe for 4S to 16S LiPo/Li-ion). Voltage spikes may not exceed 75V
- 12V 1A switchable output for external electronics
- 5V 1A output for external electronics
- 3.3V 0.5A output for external electronics
- Combined 3.3V, 5V, 12V: no more than 1A

**Current**
- Continuous 300A, burst 400A (values depend on the mounting, ambient temperature and air/water circulation around the device)

**Motor control modes**
- DC, BLDC, FOC (sinusoidal). This winch uses **FOC**.
- Sensored, sensorless or hybrid operation
- Sensorless modes: HFI, VSS, 45 Deg V0V7 HFI (Silent), 45 Deg V0 HFI, Coupled V0V7 HFI (Silent), Coupled V0 HFI
- High ERPM drivable: 100–150K (motor/system dependent)

**Supported sensors**
- Hall sensors
- Encoders: ABI, AS5047, AS5X47U, SIN/COS, TS5700N8501 (incl. multiturn), MT6816, BISSC, TLE5102, custom encoder

**Communication**
- USB, SWD
- PWM in/out (this winch: PPM input from the receiver on the "Servo" port)
- UART x2 (this winch: telemetry to the receiver on the COMM port)
- SPI and I²C
- CAN, UAVCAN and custom CAN commands
- Wireless connectivity via accessory (WiFi/BLE)
- 2x programmable GPIO pins

**Other technical features**
- Current and voltage measurement on all phases (3 phase shunts), adjustable current and voltage filters (full phase filters)
- 3 individual gate drivers
- Accelerometer and gyro (9 axis, ±2/±4/±8/±16 g full scale)
- Hibernation with wake-up via power switch options (momentary NC), automatic hibernation with adjustable timer, 20µA consumption while hibernating (less than battery self-discharge)

**Software**
- VESC Tool: https://vesc-project.com/vesc_tool (desktop), mobile apps for Android/iOS
- Motor and input setup wizards
- Scripting support (QML and LISP)

**Housing**
- Precision CNC aluminium heat sink housing, black hard anodised
- Mounting holes for easy attachment
- Outer dimensions: 141 x 82 x 18 mm

*Source: Trampa product page (manufacturer data), retrieved September 2026.*

# VESC setup (VESC Tool)

The VESC is configured with **VESC Tool** (https://vesc-project.com/vesc_tool) over USB. This section explains the setup steps and the background needed to understand them.

> ⚠ **Firmware:** The VESC Tool version must fit the firmware on the VESC (most likely FW 5.x, see [WINCH-08](../features/WINCH-08-autostop-docs-verification.md)). A newer VESC Tool may offer a **firmware update: decline it.** An update would replace the patched autostop firmware ([vesc_75_300_auto_stop.bin](vesc_75_300_auto_stop.bin)) with a stock firmware without line autostop.
>
> **Backups:** Before and after every change, save the motor and app configuration as XML files in VESC Tool and commit them to this folder with a date prefix, e.g. `260929_motor_config.xml`.

## Background: BLDC vs. FOC

The QS hub motor has a rotating outer ring with permanent magnets and fixed copper coils inside. The VESC sends current through the coils. The resulting magnetic field pulls on the magnets and turns the ring. To do this right, the VESC must know at every moment **where the magnets are** (the rotor position), so it can energise the right coil at the right time. With wrong timing the motor pulls weaker, stutters or twitches backwards.

BLDC and FOC are two ways of driving the coils:

| | BLDC (block commutation) | FOC (Field Oriented Control) |
|---|---|---|
| How | Switches the coils hard on and off in 6 fixed steps per electrical revolution | Controls the current in all three coils continuously as smooth sine waves, so the field always pulls at the optimal angle |
| Picture | Pushing a swing with single hard shoves | Guiding the swing smoothly all the way |
| Result | Simple and robust, but louder, slightly pulsing torque, less efficient | Quiet, smooth, efficient, **precise torque control** |

**The winch uses FOC** (`motor_type = 2` in all configs in this folder), and that is the right choice: the pull on the line is proportional to the motor current (torque = current × motor constant). The pull states from the transmitter end up as current commands, and FOC turns them into a smooth, non-pulsing pull.

## Background: sensorless vs. hall sensors

BLDC or FOC is about *how* the coils are driven. A separate question is *how the VESC knows the rotor position*.

**Sensorless:** A turning motor induces a voltage in its coils (back-EMF). The VESC measures it and calculates the rotor position from it (the "observer"). The problem: this voltage is proportional to speed. At standstill it is zero, at very low speed it is tiny, so the VESC cannot "see" the rotor well. When starting it has to guess. Without load this usually works; **under load it can stutter, jerk or briefly turn the wrong way.** HFI (high frequency injection) is a workaround that finds the position with an audible test signal and needs careful tuning.

**Hall sensors:** Three small magnetic field sensors inside the motor report a coarse rotor position (6 sectors per electrical revolution) over the sensor cable, **also at standstill.** In FOC hall mode the VESC uses the hall signals below `foc_sl_erpm` and switches to the more precise sensorless observer above it.

What this means on the winch (rough numbers from the config: 32 magnets = 16 pole pairs, drum diameter 0.43 m):

- `foc_sl_erpm = 2000` ERPM ÷ 16 ≈ **125 rpm of the drum ≈ 2.8 m/s (≈ 10 km/h) line speed.** Below that, hall mode uses the sensors.
- That is exactly where a winch works a lot, **at standstill or slowly, under high load:** pre-tensioning with the pilot still standing, the transition from brake to pull, braking to standstill (autostop, end of rewind), and the start of the rewind.
- With hall sensors these phases are smooth and predictable. Sensorless they can be jerky, depending on motor and tuning.

Things to know:

- **The 6-wire sensor cable also carries the motor temperature sensor** (5 V, GND, H1, H2, H3, TEMP). With the cable unplugged, the VESC has **no motor over-temperature protection** (`l_temp_motor_start/end`), and the remote cannot show a real motor temperature.
- **Downside of hall mode:** the VESC depends on the sensor cable. A loose plug, broken wire or faulty sensor gives wrong positions: jerks, weak pull or fault codes. Cable and connectors must be reliable. Sensorless mode has no such dependency.
- **Plugging in the cable alone changes nothing.** The VESC only uses the sensors when the sensor mode is set to *Hall* **and** the hall table has been measured (see below).

Sensor mode in the configs in this folder:

| Config | Origin | `foc_sensor_mode` | Hall table |
|---|---|---|---|
| [vesc_motor_config_12kw_260_V4.xml](vesc_motor_config_12kw_260_V4.xml) | Robert Zach, 2022 | **2 = Hall** | measured |
| [vesc_motor_config_12kw_273.xml](vesc_motor_config_12kw_273.xml) | Robert Zach, 2022 | **2 = Hall** | measured |
| [240601_motor_config.xml](240601_motor_config.xml) | Etienne, June 2024 | **0 = Sensorless** | empty (all 255) |

Which mode is active on the VESC right now has to be read in VESC Tool: **Motor Settings → FOC → General → Sensor Mode.**

## Recommendation: connect and use the hall sensors

> ⚠ **Safety-relevant.** This changes how the motor starts and brakes. Bench test first, then field test. Tracked in [WINCH-17](../features/WINCH-17-hall-sensors-foc.md).

Recommended because the winch spends its most critical moments (start, pre-pull, braking, autostop) at low speed under load, and because the sensor cable also restores the motor temperature protection.

**Before you start**
1. Find out why the VESC showed a fault when the repaired sensor cable was connected last time ([WINCH-16](../features/WINCH-16-uart-telemetry-zero.md)): read the fault code in VESC Tool.
2. Check the sensor cable wire by wire (5 V, GND, H1, H2, H3, TEMP) against the motor's pinout and the VESC sensor port. Check the sensor supply voltage the motor's hall sensors need. Test procedure for the temperature sensor and the three hall sensors (German): [doc/qs-motor/sensors-troubleshooting.md](../doc/qs-motor/sensors-troubleshooting.md). Wire colours: [doc/qs-motor/README.md](../doc/qs-motor/README.md) (plug colours differ from motor wire colours). *Status 2026-09-30:* hall sensors of sensor set 1 measured intact, temperature sensor 0.97 kΩ at ~25 °C (KTY83-122); the VESC fault from step 1 is still open.
3. Find out which temperature sensor the QS motor has (e.g. from the QS order/spec sheet) and set `m_motor_temp_sens_type` to match. The configs differ here: Robert's uses `2`, Etienne's 2024 config `0`.

**Procedure**
1. Save the current motor and app configuration as XML (backup).
2. Secure the winch. **Unhook the line or make sure there is no load on it, and keep everyone away from the drum: the motor turns by itself during detection.**
3. Plug in the sensor cable.
4. Run **Setup Motor FOC** again (or, in Motor Settings → FOC → Hall Sensors, run only the hall detection).
5. Check the hall table: entries 1–6 must have values, only entries 0 and 7 are 255. If more entries are 255, a sensor or wire is faulty. Do not use hall mode then.
6. Set **Sensor Mode = Hall**, write the motor configuration, and check the other motor settings against the backup (see "Setup Motor FOC" below).
7. Save the new configuration as XML and commit it.

**Bench checks** (workshop, no pilot)
- Smooth start from standstill in the pull states, also against load (line held / tied back), no jerks or backwards twitch.
- Brake to standstill (soft brake, hard brake) without jerks.
- Motor temperature on the remote is plausible (about ambient temperature when cold).
- Line length counts up and down ([WINCH-16](../features/WINCH-16-uart-telemetry-zero.md)).
- Unplug the sensor cable deliberately once on the bench (low pull state only) to see how the VESC reacts (fault code, behaviour), so the failure mode is known.

## Setup wizards

VESC Tool has two wizards on the welcome page: **Setup Motor FOC** and **Setup Input**.

### Setup Motor FOC

The wizard asks roughly for:
- the motor class (e.g. large hub motor),
- the battery (type, number of cells: 16, capacity, current limits),
- drive setup (wheel/drum diameter, motor poles: 32, gear ratio 1),

and then **measures the motor**. The motor makes noises and **turns during the measurement** (no line load, nobody at the drum). Afterwards it lets you choose the motor direction.

What it measures and calculates (values from [240601_motor_config.xml](240601_motor_config.xml)):

| Value | Config name | Value in backup | What it means in simple words |
|---|---|---|---|
| Resistance R | `foc_motor_r` | 4.34 mΩ | Electrical resistance of the copper windings. Determines the heat losses (heat = I² × R: at 200 A about 170 W in the windings) and is needed for current control. |
| Inductance L | `foc_motor_l` | 17.8 µH | How "sluggish" the windings are against changes in current. Determines how fast the VESC can change the current. |
| Flux linkage λ | `foc_motor_flux_linkage` | 26.1 mWb | Strength of the magnets as seen by the coils. The key motor constant: it links **current to torque** and **speed to voltage**. |
| Current controller gains | `foc_current_kp`, `foc_current_ki` | 0.0178 / 4.34 | Settings of the VESC's internal current regulator. Not measured but **calculated from L and R** (here L × 1000 and R × 1000). |
| Observer gain | `foc_observer_gain` | 1.47 × 10⁶ | Tuning of the sensorless position estimator, **calculated from λ**. |
| Hall table | `foc_hall_table__0…7` | all 255 (no sensors) | Only with hall sensors: which rotor angle each of the 6 sensor combinations stands for. |

Rough cross-check with λ (theoretical, ignoring friction and losses): torque per amp ≈ 1.5 × 16 pole pairs × 0.0261 ≈ 0.63 Nm/A. At a drum radius of 0.215 m that is ≈ 2.9 N per amp, i.e. **≈ 3.4 A per kg of line pull.** This fits the ≈ 3.6 A/kg estimate used for the pull states (see [WINCH-14](../features/WINCH-14-pull-value-calibration.md)). The effective drum radius grows as line is wound on, so the real value varies.

After the wizard, **compare the motor settings with the last backup.** The wizard sets current limits and other values based on the chosen motor class and battery, and these can differ from the values the winch was set up with (table below).

### Setup Input

Configures how the VESC reads the PPM signal from the LoRa receiver. The receiver outputs pulses between 950 µs (hard brake side) and 2000 µs (full pull side). In VESC Tool these are the pulse start / center / end values:

| Config | Pulse start | Center | End |
|---|---|---|---|
| [240601_app_config.xml](240601_app_config.xml) (Etienne) | 0.933 ms | 1.458 ms | 1.984 ms |
| [vesc_app_config.xml](vesc_app_config.xml) (Robert) | 1.1 ms | 1.459 ms | 1.768 ms |

> ⚠ **Do not re-run the input wizard casually.** The pulse range defines how many amps each pull state means, i.e. the actual pull in kg. Changing it is safety-relevant, needs Etienne's decision and a bench test, and should go together with the pull calibration ([WINCH-14](../features/WINCH-14-pull-value-calibration.md)).

## Motor Settings

Motor settings are specific to the motor. **Every time a different motor is connected, the motor must be set up again** (Setup Motor FOC), otherwise the VESC and/or the motor can be damaged. Changes only take effect after the motor configuration has been **written** to the VESC.

Settings worth knowing (values from [240601_motor_config.xml](240601_motor_config.xml); the VESC itself may differ):

| Setting | Config name | Value in backup | Note |
|---|---|---|---|
| Motor current max / min | `l_current_max` / `l_current_min` | 250 A / −250 A | Upper limit for pull and brake torque. |
| Battery current max / min | `l_in_current_max` / `l_in_current_min` | 290 A / −290 A | Max. current drawn from / fed back into the battery. Must fit the BMS. |
| Absolute max current | `l_abs_current_max` | 290 A | Hard cut-off: above this the VESC faults. |
| Battery cutoff start / end | `l_battery_cut_start` / `l_battery_cut_end` | 54.4 V / 48 V (3.4 / 3.0 V per cell) | Below "start" the VESC reduces the current, at "end" it stops. The battery voltage sags under load, so with a partly discharged battery the **pull can be reduced during a tow**. |
| Max input voltage | `l_max_vin` | 72 V | Over-voltage fault. Full battery = 67.2 V; regenerative braking raises it. |
| Motor temp start / end | `l_temp_motor_start` / `l_temp_motor_end` | 75 °C / 85 °C | Current is reduced between these. **Only works with the temperature sensor connected.** |
| FET temp start / end | `l_temp_fet_start` / `l_temp_fet_end` | 85 °C / 100 °C | Controller temperature; see cooling fan. |
| Motor type | `motor_type` | 2 = FOC | |
| Sensor mode | `foc_sensor_mode` | 0 = Sensorless | 2 = Hall, see recommendation above. |
| Sensorless switch-over speed | `foc_sl_erpm` | 2000 ERPM | Below this, hall mode uses the sensors. |
| Motor temp sensor type | `m_motor_temp_sens_type` | 0 | Must match the motor's sensor (open question). |
| Motor poles | `si_motor_poles` | 32 | 16 pole pairs. |
| Wheel (drum) diameter | `si_wheel_diameter` | 0.43 m | Used for speed/distance in VESC Tool. |
| Battery cells / capacity | `si_battery_cells` / `si_battery_ah` | 16 / 40 Ah | 16S10P × 4 Ah. |

## App Settings

App settings define which input the VESC listens to. Changes only take effect after the app configuration has been **written** to the VESC.

| Setting | Config name | Value in backup | Note |
|---|---|---|---|
| App to use | `app_to_use` | 4 = PPM and UART | PPM = pull command from the receiver, UART = telemetry to the receiver. |
| UART baud rate | `app_uart_baudrate` | 115200 | Must match `receiver.ino`. |
| PPM control type | `app_ppm_conf.ctrl_type` | 3 | "Current, no reverse, with brake" (FW 5.x numbering; confirm in VESC Tool). |
| PPM ramp up / down | `app_ppm_conf.ramp_time_pos` / `_neg` | 1 s / 0.5 s | VESC-side smoothing, on top of the receiver's own smoothing. |
| Timeout | `timeout_msec` / `timeout_brake_current` | 1000 ms / 10 A | If the PPM signal is missing for 1 s, the VESC brakes with 10 A. |

The potentiometer on ADC2 is handled by the autostop firmware patch, not by a normal app (see "Line auto stop in VESC" below).

**Toolbar:** On the right-hand side of VESC Tool there are buttons for motor (**M**) and app (**A**) configuration: *read* the configuration from the VESC, *read default* configuration, and *write* the configuration to the VESC. Always read first, change, then write.

## Config backups in this repo

| File | Origin | Content |
|---|---|---|
| [240601_motor_config.xml](240601_motor_config.xml) | Etienne, 2024-06-01 | Motor config of this winch (sensorless) |
| [240601_app_config.xml](240601_app_config.xml) | Etienne, 2024-06-01 | App config of this winch |
| [vesc_motor_config_12kw_260_V4.xml](vesc_motor_config_12kw_260_V4.xml) | Robert Zach, 2022 | QS 12 kW 260 V4, hall mode |
| [vesc_motor_config_12kw_273.xml](vesc_motor_config_12kw_273.xml) | Robert Zach, 2022 | QS 12 kW 273, hall mode |
| [vesc_app_config.xml](vesc_app_config.xml) | Robert Zach, 2022 | App config |

**PLEASE NOTE:** Don't just load a motor config and write it to your VESC. Use it as an example only. Always run the **Setup Motor FOC** wizard so the VESC measures the real motor (resistance, inductance, flux linkage, hall sensors).

## Line auto stop in VESC
Line auto stop can be implemented within VESC with vesc_ppm_auto_stop.patch

For this to work properly, either connect a Potentiometer to ADC2 and GND to manually control the winch. E.g. To wind up the last meters of the line when finishing. Or to manually set a tension when used as a rewind winch. Note that the potentiometer only reduces tension/speed of the motor when it is running one of the pull programs as controlled via the transmitter!

IMPORTANT: If you do not install a Potentionmeter, connect ADC2 to GND.

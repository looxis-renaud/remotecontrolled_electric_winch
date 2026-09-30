# WINCH-17: Reconnect hall & motor temperature sensors, FOC hall mode

**Status:** Bench Test
**Type:** HW / VESC (config) / Doc
**Priority:** P1
**Safety-relevant:** Yes (changes how the motor starts and brakes at low speed under load; restores motor over-temperature protection)
**Depends on:** WINCH-16
**Created:** 2026-09-29 · **Last updated:** 2026-09-30

## Motivation
Until 2026-09-30 the winch ran FOC **sensorless** (June 2024 config: `foc_sensor_mode = 0`, hall table empty), with the 6-wire sensor cable (hall sensors + motor temperature) unplugged after a VESC error (see WINCH-16). Robert's original configs used hall mode.

Sensorless FOC cannot see the rotor position at standstill and at very low speed. That is exactly where the winch does its most critical work: pre-tensioning, brake ↔ pull transitions, braking to standstill (autostop, end of rewind). With `foc_sl_erpm = 2000` and 16 pole pairs, hall mode uses the sensors below ≈125 rpm ≈ 2.8 m/s line speed. Without the sensor cable the VESC also had no motor over-temperature protection.

Background and procedure: [vesc/readme.md](../vesc/readme.md) ("Background: sensorless vs. hall sensors", "Hall sensors: how they were set up").

## Scope
1. [x] Find the cause of the VESC fault with the sensor cable connected → `FAULT_CODE_OVER_TEMP_MOTOR`, wrong temperature sensor type (see Log).
2. [x] Check the sensor cable wire by wire. Test procedure (German): [doc/qs-motor/sensors-troubleshooting.md](../doc/qs-motor/sensors-troubleshooting.md). Wire colours: [doc/qs-motor/README.md](../doc/qs-motor/README.md). Hall sensors of set 1 intact.
3. [x] Determine the motor's temperature sensor type and set `m_motor_temp_sens_type` accordingly → KTY83/122.
4. [x] Hall detection (hall detection only, not the wizard), check the hall table, set sensor mode to Hall.
5. [x] Save the XML backups → `vesc/260930_motor_config.xml`, `vesc/260930_app_config.xml`. Older backups moved to `old/vesc-configs/`.

## Out of scope
- VESC firmware changes (frozen).
- PPM input mapping / pull values (WINCH-14).
- The over-voltage fault when braking from full speed → [WINCH-22](WINCH-22-overvoltage-regen-braking.md).

## Changes
- **Hardware:** sensor cable (set 1) connected.
- **VESC:** motor config only: `m_motor_temp_sens_type` 0 (NTC 10k) → 2 (KTY83/122); hall table measured; `foc_sensor_mode` 0 (sensorless) → 2 (Hall). Everything else unchanged (checked by comparing the XML with the June 2024 backup: only these values and the automatic current/voltage offsets differ). Firmware unchanged (FW 5.3).
- **Docs:** `vesc/readme.md`, new XML backups, `doc/qs-motor/README.md`.

## Test plan
### Bench (workshop, no pilot)
- [x] Hall table: entries 1–6 valid, only 0 and 7 = 255 → `255, 65, 118, 103, 199, 34, 173, 255` (2026-09-30).
- [x] With the potentiometer: smooth start from standstill and smooth braking from slow speed (Etienne, 2026-09-30).
- [ ] Smooth start from standstill **in the pull states** (remote), also against load, no jerks or backwards twitch.
- [ ] Soft and hard brake to standstill without jerks (from low speed only until WINCH-22 is resolved).
- [ ] Motor temperature: VESC Tool showed plausible values (`T Motor` 43.6 °C after a test run, 2026-09-30). Still open: heating test (rises when warmed) and the value on the remote.
- [ ] Line length counts on the remote (WINCH-16). The VESC-internal tachometer counts.
- [ ] Sensor cable deliberately unplugged once (low pull state): VESC reaction noted here. Expected: `FAULT_CODE_OVER_TEMP_MOTOR`, no current.
- [x] Motor settings compared with the previous backup (current limits, cutoffs, temperature limits unchanged; no wizard used).

### Field (real towing)
- [ ] Several tows incl. pre-pull, step towing, release, rewind (max. state 2) and autostop: behaviour noted here.

## Open questions
- [x] Which fault code did the VESC show with the sensor cable connected? → `FAULT_CODE_OVER_TEMP_MOTOR`.
- [x] Which temperature sensor does the QS 12kW 260 V4 have? → KTY83-122 (0.97 kΩ at ~25 °C, QS manual; VESC readings plausible with the KTY83/122 setting). Heating test still open.
- [x] Which motor config is currently on the VESC? → Until 2026-09-30 practically identical to the June 2024 backup (sensorless, NTC 10k). Now: `vesc/260930_motor_config.xml` (hall mode, KTY83/122).

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Temperature sensor type KTY83/122 | Measured 0.97 kΩ at ~25 °C, QS manual; NTC setting caused a permanent over-temperature fault | 2026-09-30 |
| Hall detection only, not the Setup Motor FOC wizard | The wizard would also reset current limits, battery and temperature settings | 2026-09-30 |
| FOC hall mode | Smooth start/braking at standstill and low speed under load | 2026-09-30 |

## Log
- 2026-09-29: created. Background, recommendation and procedure written in vesc/readme.md.
- 2026-09-29: sensor test procedure added as doc/motor-sensors-troubleshooting.md (German, at Etienne's request).
- 2026-09-30: motor docs moved to doc/qs-motor/ (sensor test procedure now doc/qs-motor/sensors-troubleshooting.md, new motor data sheet doc/qs-motor/README.md).
- 2026-09-30 (Etienne): temperature sensor (sensor set 1) measured 0.97 kΩ at ~25 °C, matches KTY83-122. Not yet heated to confirm the rising resistance.
- 2026-09-30 (Etienne): all three hall sensors of sensor set 1 (own plug) checked by measurement: intact. Wire colour mapping motor ↔ plug recorded in doc/qs-motor/README.md (plug colours differ from motor colours). Sensor set 2 (original QS plug) never connected, unchecked.
- 2026-09-30 (VESC Tool 3.01 session with Etienne, FW 5.3 confirmed):
  - Sensor cable connected → red LED; `faults`: `FAULT_CODE_OVER_TEMP_MOTOR`. Cause: `m_motor_temp_sens_type = 0` (NTC 10k); the KTY83's ~1 kΩ reads as ~100 °C, above the 75/85 °C limits → no current, the motor didn't turn even with the potentiometer.
  - Backups taken, sensor type set to KTY83/122 (Motor Settings → General → Advanced), written. Motor first jittered with the potentiometer; after a VESC restart no faults and the potentiometer worked normally (sensorless).
  - Test at full speed with the potentiometer, then fast braking → `FAULT_CODE_OVER_VOLTAGE` (fault at 72.1 V, Realtime Data showed up to 88 V) → [WINCH-22](WINCH-22-overvoltage-regen-braking.md).
  - Hall detection run (Hall Sensors tab only): table `255, 65, 118, 103, 199, 34, 173, 255`. Sensor mode set to Hall, written. Potentiometer test: smooth start and smooth braking from slow speed.
  - New backups `vesc/260930_motor_config.xml` / `vesc/260930_app_config.xml`; old backups moved to `old/vesc-configs/`. Status → Bench Test.

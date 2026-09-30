# WINCH-17: Reconnect hall & motor temperature sensors, FOC hall mode

**Status:** Planned
**Type:** HW / VESC (config) / Doc
**Priority:** P1
**Safety-relevant:** Yes (changes how the motor starts and brakes at low speed under load; restores motor over-temperature protection)
**Depends on:** WINCH-16
**Created:** 2026-09-29 · **Last updated:** 2026-09-30

## Motivation
The winch currently runs FOC **sensorless** (Etienne's backup `vesc/240601_motor_config.xml`: `foc_sensor_mode = 0`, hall table empty), and the 6-wire sensor cable (hall sensors + motor temperature) is unplugged (see WINCH-16). Robert's original configs used hall mode.

Sensorless FOC cannot see the rotor position at standstill and at very low speed. That is exactly where the winch does its most critical work: pre-tensioning, brake ↔ pull transitions, braking to standstill (autostop, end of rewind). With `foc_sl_erpm = 2000` and 16 pole pairs, hall mode would use the sensors below ≈125 rpm ≈ 2.8 m/s line speed. Without the sensor cable the VESC also has no motor over-temperature protection.

Background and procedure are documented in [vesc/readme.md](../vesc/readme.md) ("Background: sensorless vs. hall sensors", "Recommendation: connect and use the hall sensors").

## Scope
1. Find the cause of the VESC fault that appeared when the repaired sensor cable was connected (fault code, WINCH-16).
2. Check the sensor cable wire by wire (5 V, GND, H1, H2, H3, TEMP) and the sensor supply voltage. Test procedure (German): [doc/qs-motor/sensors-troubleshooting.md](../doc/qs-motor/sensors-troubleshooting.md). Wire colours: [doc/qs-motor/README.md](../doc/qs-motor/README.md). Hall sensors of set 1 measured intact (2026-09-30).
3. Determine the motor's temperature sensor type and set `m_motor_temp_sens_type` accordingly.
4. Hall detection (Setup Motor FOC or hall detection only), check the hall table, set sensor mode to Hall.
5. Save before/after XML backups to `vesc/` with a date prefix.

## Out of scope
- VESC firmware changes (frozen).
- PPM input mapping / pull values (WINCH-14).

## Changes
- **Hardware:** sensor cable reconnected (and repaired if needed).
- **VESC:** motor config only (sensor mode, hall table, temp sensor type). Firmware unchanged.
- **Docs:** `vesc/readme.md` setup section (written 2026-09-29), new XML backups.

## Test plan
### Bench (workshop, no pilot)
- [ ] Hall table: entries 1–6 valid, only 0 and 7 = 255.
- [ ] Smooth start from standstill in the pull states, also against load, no jerks or backwards twitch.
- [ ] Soft and hard brake to standstill without jerks.
- [ ] Motor temperature on the remote plausible.
- [ ] Line length counts (WINCH-16).
- [ ] Sensor cable deliberately unplugged once (low pull state): VESC reaction noted here.
- [ ] Motor settings compared with the previous backup after the wizard (current limits, cutoffs, temperature limits).

### Field (real towing)
- [ ] Several tows incl. pre-pull, step towing, release, rewind (max. state 2) and autostop: behaviour noted here.

## Open questions
- [ ] Which fault code did the VESC show with the sensor cable connected?
- [ ] Which temperature sensor does the QS 12kW 260 V4 have? (Robert's config: type 2, Etienne's 2024 config: type 0.) Most likely KTY83-122 (measured 0.97 kΩ at ~25 °C, QS manual says KTY83-122); heating test still open.
- [ ] Which motor config is currently on the VESC (sensor mode)?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-09-29: created. Background, recommendation and procedure written in vesc/readme.md.
- 2026-09-29: sensor test procedure added as doc/motor-sensors-troubleshooting.md (German, at Etienne's request).
- 2026-09-30: motor docs moved to doc/qs-motor/ (sensor test procedure now doc/qs-motor/sensors-troubleshooting.md, new motor data sheet doc/qs-motor/README.md).
- 2026-09-30 (Etienne): temperature sensor (sensor set 1) measured 0.97 kΩ at ~25 °C, matches KTY83-122. Not yet heated to confirm the rising resistance. If confirmed, `m_motor_temp_sens_type` has to be set to the KTY83 option in VESC Tool (config change, needs Etienne's confirmation).
- 2026-09-30 (Etienne): all three hall sensors of sensor set 1 (own plug) checked by measurement: intact. Wire colour mapping motor ↔ plug recorded in doc/qs-motor/README.md (plug colours differ from motor colours). Sensor set 2 (original QS plug) never connected, unchecked.

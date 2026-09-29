# WINCH-17: Reconnect hall & motor temperature sensors, FOC hall mode

**Status:** Planned
**Type:** HW / VESC (config) / Doc
**Priority:** P1
**Safety-relevant:** Yes (changes how the motor starts and brakes at low speed under load; restores motor over-temperature protection)
**Depends on:** WINCH-16
**Created:** 2026-09-29 · **Last updated:** 2026-09-29

## Motivation
The winch currently runs FOC **sensorless** (Etienne's backup `vesc/240601_motor_config.xml`: `foc_sensor_mode = 0`, hall table empty), and the 6-wire sensor cable (hall sensors + motor temperature) is unplugged (see WINCH-16). Robert's original configs used hall mode.

Sensorless FOC cannot see the rotor position at standstill and at very low speed. That is exactly where the winch does its most critical work: pre-tensioning, brake ↔ pull transitions, braking to standstill (autostop, end of rewind). With `foc_sl_erpm = 2000` and 16 pole pairs, hall mode would use the sensors below ≈125 rpm ≈ 2.8 m/s line speed. Without the sensor cable the VESC also has no motor over-temperature protection.

Background and procedure are documented in [vesc/readme.md](../vesc/readme.md) ("Background: sensorless vs. hall sensors", "Recommendation: connect and use the hall sensors").

## Scope
1. Find the cause of the VESC fault that appeared when the repaired sensor cable was connected (fault code, WINCH-16).
2. Check the sensor cable wire by wire (5 V, GND, H1, H2, H3, TEMP) and the sensor supply voltage.
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
- [ ] Which temperature sensor does the QS 12kW 260 V4 have? (Robert's config: type 2, Etienne's 2024 config: type 0.)
- [ ] Which motor config is currently on the VESC (sensor mode)?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-09-29: created. Background, recommendation and procedure written in vesc/readme.md.

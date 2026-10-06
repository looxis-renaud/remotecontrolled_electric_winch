# WINCH-16: Line length & duty cycle stay at 0 (UART telemetry)

**Status:** Planned
**Type:** FW / VESC (read-only) / HW
**Priority:** P0
**Safety-relevant:** Yes (the receiver-side autostop taper uses the UART tachometer; the pilot/operator relies on the line length)
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-10-03

## Motivation
The transmitter's bottom OLED line (`<line length>m| <duty cycle>%`) always shows 0 / 0 (Etienne, 2026-09-28). The values come from the VESC via UART to the receiver, then via LoRa ack to the transmitter. The motor's temperature sensor and hall sensors are currently not connected to the VESC. Unclear whether (a) the UART link is broken, or (b) UART works but the VESC reports 0.

This must be clarified before WINCH-08 (autostop verification), because autostop depends on the tachometer: the receiver taper directly via UART, the VESC autostop via the VESC's internal tachometer.

## What the code does (read, not tested)
- Receiver reads the VESC every 20 loops (`vescUART.getVescValues()`, `Serial1` 115200 baud, RX = IO14, TX = IO2). If the read fails, the old values stay, and they start at 0.
- Ack to the transmitter: `tachometer = abs(tacho) / 1000` (→ unit of 10 m, displayed as `×10 m`), `dutyCycleNow = abs(duty × 100)` %.
  - So the line length shows **0 until about 10 m** are unwound (if 1500 counts ≈ 15 m, as in the patch).
  - Duty cycle is **0 whenever the motor isn't turning**, by definition.
- Battery % (`B: xx%`) and motor temperature (`T: yy C`) come through **the same UART read** and alternate in the top line of the transmitter OLED.

## Findings so far (Etienne, 2026-09-28)
- **UART works:** the transmitter shows `B: 45%`. Battery % comes from the same UART read (`COMM_GET_VALUES`) as tachometer and duty cycle.
- **History of the sensor cable:**
  - Before: wiring was a mess and one wire of the sensor cable (hall or motor temp) was broken. Line length **still showed and worked** then.
  - New case, wiring cleaned up, sensor cable (6 wires: hall sensors + motor temp) connected with the broken wire fixed → **VESC showed an error** → sensor cable unplugged again. Currently: no hall / temp sensors connected.
- **Tachometer stopped about two days ago (~2026-09-26):** turning the motor with the potentiometer (manual mode) used to make the line length count up; now it stays 0. Duty cycle also stays 0.
- **Autostop still seems to work.**

### Interpretation (from reading the patch, not verified)
- The patch applies autostop in normal PPM mode whenever `abs(tachometer) < 1500`: 18 A brake, PPM input ignored (`vesc_ppm_auto_stop.patch` lines 45-48). If the VESC-internal tachometer were really stuck at 0, the winch would **only brake and never pull** in PPM mode.
- So if the winch still pulls normally *and* autostop still triggers, the VESC-internal tachometer probably still counts, and only the value that reaches the receiver/transmitter is 0. That would point to parsing / data path (e.g. VescUart library vs. firmware packet layout) rather than to the missing hall sensors.
- Counter-check needed: if tachometer *and* duty cycle are 0 but voltage is fine, the packet is received but some fields are read wrongly or not updated.

### Questions for the next debug session
- [ ] What changed around two days ago? Receiver reflashed? Arduino libraries updated (VescUart!)? VESC reflashed or reconfigured (e.g. motor detection / sensor mode changed while connecting the sensor cable)?
- [x] Which error did the VESC show with the sensor cable connected (fault code in VESC Tool)? → `FAULT_CODE_OVER_TEMP_MOTOR`, wrong temperature sensor type (2026-09-30, see Log).
- [ ] How exactly was "autostop still works" observed (PPM mode or poti mode, which line length)? Note: in poti mode autostop is only active if ADC1 > 3 V (see WINCH-08).
- [ ] In PPM mode (remote), does the winch pull normally after line has been pulled out, or does it only brake?
- [ ] What does the motor temperature `T:` show (expected: 0 or an implausible value without sensor)?
- [ ] VESC Tool → Realtime data: tachometer and duty cycle while the motor turns. Do they count there?

## Scope: diagnosis first (no code / VESC changes)
1. [x] **Top line of the transmitter OLED:** does `B:` show a plausible battery % (not 0)? → Yes, 45 % (2026-09-28). Step 2 not needed for now.
   - Yes → UART works; the zeros are a VESC-side / measurement issue (continue at 3).
   - No (`B: 0%`) → UART read fails (continue at 2).
2. [ ] **If UART fails:**
   - Wiring: VESC COMM TX → receiver IO14, VESC COMM RX → receiver IO2, common GND.
   - VESC Tool → App settings: app = "PPM and UART", UART baud 115200. (Backups `old/vesc-configs/240601_app_config.xml` / `vesc_app_config.xml` have `app_to_use = 4` = PPM + UART, 115200. To confirm on the real VESC.)
   - Known pin collision: `RST 14` (LoRa reset) = `VESC_RX 14`. `LoRa.begin()` runs *after* `Serial1.begin()` and may reconfigure IO14. Suspect, not confirmed (see WINCH-15).
   - Optional: enable `vescUART.setDebugPort(&Serial)` / the commented-out serial prints in `receiver.ino` for a bench session (temporary, not committed).
3. [ ] **If UART works but values stay 0:**
   - Check with the motor actually turning under pull (states ≥1, line being pulled in/out). At standstill 0 / 0 is expected.
   - VESC Tool → Realtime data: does the tachometer change while the drum turns (a) under pull, (b) when line is pulled out by hand under soft brake?
   - Hall sensors: June 2024 backup (now `old/vesc-configs/240601_motor_config.xml`) has `foc_sensor_mode = 0` (sensorless; hall mode active since 2026-09-30, WINCH-17); the older configs have `2` (hall). In sensorless FOC the tachometer is counted by the observer **while the motor is driven**; at low speed / by hand it may not count. To confirm on the real VESC which mode is active.
4. [ ] Record the result here, then decide the fix (wiring, pin change in `receiver.ino` → safety-relevant, or reconnecting hall sensors).

## Out of scope
- VESC firmware / config changes (frozen; only if Etienne explicitly decides so).
- The known ~0.7× line-length offset and the tachometer unit mismatch (receiver taper 2–40 vs. patch 1500) → WINCH-08, unless the diagnosis shows they are the same problem.

## Changes
- To be decided after diagnosis.

## Test plan
### Bench (workshop, no pilot)
- [ ] Transmitter shows plausible battery % and motor temp (or a known "no sensor" value).
- [ ] Line length on the transmitter increases while line is unwound, and returns towards 0 when rewound.
- [ ] Duty cycle shows a non-zero value while the drum turns.

## Open questions
- [x] Did line length / duty cycle ever work with the current setup? → Yes, until about two days ago, even with a broken sensor wire.
- [x] Why are hall / temp sensors disconnected? → VESC showed an error when the repaired sensor cable was connected. Reconnecting depends on that error (see questions above).
- [ ] Does the VESC's own autostop still work without hall sensors? → verify in WINCH-08.

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Diagnose before WINCH-08 | Autostop verification needs a working, trustworthy tachometer | 2026-09-28 |

## Log
- 2026-09-28: created (reported by Etienne).
- 2026-09-28: first findings added: UART works (B 45 %), tacho stopped ~2 days ago, sensor cable unplugged after VESC error, autostop seems to still work. Debugging postponed.
- 2026-09-29: reconnecting the hall / temperature sensors and switching to FOC hall mode split out as [WINCH-17](WINCH-17-hall-sensors-foc.md); background in vesc/readme.md.
- 2026-09-30 (VESC Tool session with Etienne, VESC Tool 3.01, FW 5.3):
  - **The VESC error with the sensor cable connected was `FAULT_CODE_OVER_TEMP_MOTOR`**: `m_motor_temp_sens_type` was set to NTC 10k, but the motor has a KTY83-122 (0.97 kΩ at ~25 °C). As NTC this reads ~100 °C, above the 75/85 °C motor limits → fault, no current. Fixed by setting the sensor type to KTY83/122 (see WINCH-17). Without the sensor cable the input is open and now reads as extremely hot, so the cable must stay connected.
  - **The VESC-internal tachometer counts:** `faults` output showed `Tacho: 15914`, Realtime Data `Tac: 14566` / `Tac ABS: 15580` after turning the drum with the potentiometer. So the zeros are on the UART / receiver / LoRa side, not in the VESC.
  - **Transmitter OLED shows `30m| 85%` and does not update** (Etienne): at least one telemetry packet with tachometer and duty cycle got through at some point, then the values froze. Keep for debugging: frozen values rather than zeros suggest that UART reads (`getVescValues()`) fail after a first success, or the ack stops carrying new data. Lead to check first: the pin collision `RST 14` (LoRa reset) = `VESC_RX 14` in `receiver.ino`.
- 2026-10-02: according to the board pinout in `doc/` (LilyGO T3 V1.6.1), the LoRa reset is **GPIO23**, not 14. With `RST 14`, the LoRa library most likely switches IO14 to a plain output during `LoRa.begin()` (it pulses the reset pin), which runs *after* `Serial1.begin()`. The UART RX pin would then no longer receive and would drive against the VESC's TX. Not verified on the board; strong suspect for the frozen telemetry. A wiring check with the old firmware comes first (WINCH-05, "Before flashing"). Also: the board's microSD slot uses IO13/14/15/2, so no card may be inserted. Etienne: no card in the receiver, none planned.
- 2026-10-02 (bench, controller on the desk, old firmware on TX/RX): UART wiring checked by Etienne. IO14 (receiver RX) → blue wire → green wire → VESC COMM **TX**: correct. IO2 (receiver TX) → white wire, soldered to a green wire → VESC COMM **RX**: correct routing. **An orange wire measures open circuit (OL) end to end: broken.** Which signal it carries is still to be confirmed. A broken or intermittent UART wire fits the frozen values (`30m| 85%`): reads succeed while the wire makes contact, then fail and the receiver keeps sending the last values. Next: replace the wire, then check that `B:` / `T:` and line length update.
- 2026-10-02: correction: the orange wire is the IO2 → VESC **RX** line (white wire from IO2 to the orange wire of the pre-made COMM plug). Etienne found damaged insulation: several wires of the harness were stuck together. After pulling them apart the continuity test passes. Likely cause: damaged insulation / short or intermittent contact on the receiver → VESC request line (without the request the VESC never answers, so the receiver keeps the last values). Plan: re-insulate / repair the harness first, check that adjacent wires are isolated from each other, then test UART.
- 2026-10-03: harness repaired and re-insulated (Etienne); telemetry still does not work. Wiring confirmed: VESC RX ← white (IO2), VESC TX → blue (IO14). VESC app config checked in `260930_app_config.xml`: `app_to_use = 4` (PPM and UART), `app_uart_baudrate = 115200`, both fine. Main suspect now: **IO14 is both `VESC_RX` and LoRa `RST`**. `Serial1.begin()` routes IO14 to UART RX, then `LoRa.begin()` sets IO14 as OUTPUT and leaves it HIGH, so the receiver may never see the VESC's answer. The board pinout ([doc/LilyGO-TTGO-T3-LoRa32-433MHz-V2.1.6-ESP32-pinout-1500x1500w.jpg](../doc/LilyGO-TTGO-T3-LoRa32-433MHz-V2.1.6-ESP32-pinout-1500x1500w.jpg)) confirms LoRa RST = **GPIO23**. Open: why it worked earlier (receiver reflashed around 2026-09-26, possibly with a different ESP32 core?). Proposed diagnostic: old receiver code (compatible with the flashed transmitter) with only `RST 14` → `RST 23`, bench only, needs Etienne's OK.
- 2026-10-03: Etienne: the receiver was last flashed long ago and telemetry still worked afterwards, so the `RST 14` collision is probably not the only cause (the harness short may also have played a part). Code changes before reflashing both boards with the current repo version: `RST 23` (see WINCH-15); receiver prints UART diagnosis on the USB serial monitor (115200 baud) every VESC read: `VESC ok: <V> V, tacho <n>, duty <d>, motor <t> C` or `Failed to get data from VESC!`. Failsafe comment corrected (20 s, code unchanged). Both sketches compile; not flashed, not tested.
- 2026-10-03: new firmware flashed (RST 23, serial diagnosis); LoRa link works. Etienne asked for the telemetry on the receiver OLED as well: new bottom line `Tac <raw tachometer counts> D <duty cycle %> ok|ERR` (`ERR` = last `getVescValues()` failed, the values shown are then the last good ones). Shows directly on the receiver whether the UART data arrives, independent of the LoRa ack. Display only, no change to pull/brake logic. Compiles; not flashed yet.
- 2026-10-03: receiver OLED shows `ERR` (new display line): `getVescValues()` fails, while VESC Tool over USB shows Tac and duty. Cross-check by Etienne: a **second TTGO board** with a **new JST cable** (5V, GND, TX, RX) and the same receiver code also shows `ERR`, so the original receiver board and harness are unlikely to be the (only) cause. Multimeter: VESC COMM 5V → TX ≈ 1.7 V and 5V → RX ≈ 1.7 V, i.e. both ≈ 3.3 V against GND (assuming 5.0 V): normal UART idle level, no obviously dead or shorted pin. Remaining suspects: VESC UART not active / wrong setting in the live config, VESC UART damaged in a way the idle level does not show, or software (sketch / VescUart library). Next: check App Settings live, then the USB↔UART passthrough test with VESC Tool on the spare board.
- 2026-10-03: measured against GND: VESC TX and RX both 3.3 V (normal idle). VESC App Settings live (screenshot): App to Use = PPM and UART, Enable Permanent UART = True, timeout 1000 ms / 10 A brake; identical to all stored app configs since 2024 (`permanent_uart_enabled = 1`, `app_to_use = 4`, baud 115200), so no config change explains the failure. UART baud rate in the live UART tab still to be confirmed. VescUart 1.0.1 reviewed: `getVescValues()` returns false on timeout (100 ms), wrong start byte, CRC mismatch or payload ≤ 55 bytes; the library prints which one when a debug port is set. Next: spare board with `vescUART.setDebugPort(&Serial);` enabled (local diagnostic build, not committed), and/or the passthrough test.
- 2026-10-03: debug output on the spare board (`setDebugPort` enabled locally): every cycle `Package to send: 2 1 4 64 132 3` (correct COMM_GET_VALUES frame) followed by `Timeout`. **Not a single byte comes back from the VESC.** No motor connected is not the cause (the VESC answers COMM_GET_VALUES without a motor; VESC Tool over USB shows values). Next: loopback on the spare board (IO2 ↔ IO14 bridged at the cable end, VESC unplugged): expected debug output `End of message reached!` with matching SRC values, which would prove ESP32 pins + new cable and point to the VESC COMM UART.
- 2026-10-03: **loopback passed:** IO2 and IO14 bridged at the VESC end of the new JST cable → `End of message reached!`, `SRC received 16516 = SRC calc 16516`, own frame `2 1 4 64 132 3` received (then `Failed` as expected: 1-byte payload). **ESP32 UART pins and the new cable work.** With the VESC connected not a single byte comes back, so the fault is on the VESC side: TX/RX swapped at the COMM plug, or the VESC COMM UART does not receive / answer (possibly damaged by the harness short). Next: identify which COMM pin is the VESC output (TX) with a resistor load test, no swapping of outputs.
- 2026-10-03: VESC COMM TX/RX labelling confirmed correct by Etienne (and it used to work with this wiring). **Debugging paused (Etienne):** line length and duty cycle on the remote are not essential. Conclusion so far: ESP32 side and cables are fine, the VESC COMM UART does not answer (cause unknown, possibly damage from the harness short). Consequence while telemetry is missing: battery % / motor temp / line length / duty cycle on the remote are not updated, and the **receiver autostop taper is inactive** (it needs `tachometer > 2`); the VESC firmware autostop is independent of this. Possible alternative: telemetry over the unused VESC CAN port ([WINCH-25](WINCH-25-can-telemetry.md), idea). Temporary debug line re-commented in `receiver.ino`.

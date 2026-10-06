# WINCH-07: Automatic cooling fan by operating mode

**Status:** Bench Test
**Type:** FW
**Priority:** P1
**Safety-relevant:** Yes (LoRa struct change; a VESC overheating would cut power during a tow)
**Depends on:** WINCH-05, WINCH-06
**Created:** 2026-09-28 · **Last updated:** 2026-10-02

## Motivation
Today the transmitter turns the relay (fan) ON at startup, and a short press on the 3rd button toggles it. Manual toggling is unnecessary. The fan should simply run while the winch is working and stop afterwards.

## Known bug (diagnosed and fixed 2026-10-02)
**Cause:** the relay signal wire (yellow) was soldered to **GPIO15** instead of IO12. The sketch never drives GPIO15; its internal boot pull-up only made the module LED glow, too weak to switch the transistor, so the relay never clicked. Etienne resoldered it to IO12 (2026-10-02): the relay now clicks and the fan runs. Fan wiring on the load side: COM + NO (correct for `RELAY_ACTIVE_HIGH = true`). Relay module: Songle SRD-05VDC-SC-C, `+` → 5V, `-` → GND, `S` → IO12.
**Fan does not switch on, although the receiver OLED shows "Fan/Light ON"** after the remote has connected (reported by Etienne, 2026-09-28).

The OLED text only shows the software variable `relay`, so the receiver has received `relay = true` and writes IO12 HIGH every loop. The fault is therefore most likely between IO12 and the fan, not in the LoRa logic. Not yet diagnosed. Candidates to check (bench, winch not in use):
- [ ] **Relay module polarity:** many relay modules are *active-low* (IN = LOW → relay on). Then HIGH would mean OFF. Hint: does the fan run right after the receiver boots, *before* the remote is connected (IO12 LOW then)? Does the module's LED light up?
- [ ] **Trigger level:** a 5 V relay module may not switch reliably with the ESP32's 3.3 V signal. Measure IO12 against GND (expect ~3.3 V when "ON") and check whether the relay clicks.
- [ ] **Relay module supply:** 5 V and GND actually present at the module.
- [ ] **Load side:** fan wired through COM/NO (not NC), fan supply present, fan itself OK (test directly on its supply).
- [ ] **IO12 strapping pin** (see WINCH-15): check nothing on the module pulls IO12 in a way that interferes.
- [ ] Wiring vs. README "PIN Setup Receiver" note (white = signal → IO12, red → 5V, black → GND).

Whatever the cause, the new fan logic below must drive the relay with the correct polarity (possibly a named constant for active-high/low). Bench test must confirm the fan **actually runs**, not only the OLED text.

## Scope
- **Fix the bug above** (hardware/wiring and/or output polarity in `receiver.ino`).
- **Receiver** decides alone:
  - Relay (IO12) **ON** as soon as a pull state (`currentState >= 1`) is active.
  - Relay **OFF** only after `FAN_RUN_ON_MS` (**120 s**, a named constant at the top of `receiver.ino`) without any pull state. The run-on lets the VESC cool down and avoids flapping during step tows (default pull ↔ brake).
  - Failsafe states count as pull (fan stays on).
- **Transmitter:** remove the relay toggle (3rd-button short press) and the `relay` field from `LoraTxMessage` (same struct change as WINCH-05, one flash for both).
- **Receiver:** remove the `relay` field; OLED shows "Fan ON/OFF".
- README.md: describe the automatic behaviour, update the IO12 pin description.

## Out of scope
- Temperature-based fan control (possible later idea: also keep the fan on while motor/MOSFET temperature from UART is above a threshold).
- Removing the 3rd button hardware.

## Changes
- **Firmware transmitter:** remove relay handling and field.
- **Firmware receiver:** new fan logic, remove field.
- **Docs:** README.md.

## Test plan
### Bench
- [x] Known bug diagnosed and fixed: relay signal was on GPIO15, now IO12; relay clicks, fan runs (Etienne, 2026-10-02).
- [x] Power-on in soft brake: fan OFF (Etienne, 2026-10-03, new firmware; the 2026-10-02 "fan on at connect" was the old firmware).
- [x] State 1 → fan ON immediately (2026-10-03).
- [x] Back to brake: fan stays on for about 120 s, then OFF (2026-10-03).
- [ ] Brake ↔ pull within 120 s: fan stays on without switching off.
- [ ] Transmitter off during pull (failsafe): fan stays on.
- [x] Receiver boots normally with the relay connected to IO12 (strapping pin, see WINCH-15): the receiver is powered by the VESC and boots immediately when the VESC is switched on (Etienne, 2026-10-02).
### Field
- [ ] Full tow session: fan behaviour as expected, no VESC temperature problems.

## Open questions
- [x] Is 60 s run-on right, or longer (e.g. 120 s) after hard tows? → 120 s (Etienne, 2026-09-30).

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Fan ON while pull state active, OFF after run-on | Etienne's choice: no manual toggling; run-on for cooling and against flapping during step tows | 2026-09-28 |
| Logic lives in the receiver only | Receiver knows the actual state, including failsafe; no protocol field needed | 2026-09-28 |

## Log
- 2026-09-28: created
- 2026-09-28: known bug added: fan does not run although OLED shows "Fan/Light ON" (Etienne). Diagnosis checklist added.
- 2026-09-30: implemented in `receiver.ino`: fan ON while `currentState >= 1` (incl. failsafe), OFF after `FAN_RUN_ON_MS` = **120 s** (Etienne's choice) without pull state, OFF after power-up. Relay polarity as constant `RELAY_ACTIVE_HIGH` (default `true` = previous behaviour; the known bug is not diagnosed yet, set to `false` if the relay module turns out to be active-low). OLED: "Fan ON", "Fan ON (off in … s)", "Fan OFF". Transmitter: relay toggle and `relay` field removed. Compiles; not flashed, not bench-tested. Not covered: manual rewinding with the potentiometer while the remote is in brake keeps the fan off (the receiver doesn't know about poti mode).
- 2026-10-02: known bug diagnosed and fixed: relay signal wire was on GPIO15, resoldered to IO12 (Etienne). Relay clicks, fan runs. Receiver (powered by the VESC) boots normally with the relay on IO12. Open: the fan switched on as soon as the remote connected, which points to the old firmware still being flashed (see bench test plan).
- 2026-10-02: fan behaviour after release reviewed (code reading only, Etienne's observation): after release the remote stays in state 1 or 2 while the VESC AutoStop holds the drum. The receiver does not see AutoStop (it only follows the remote's state), so the fan keeps running until the remote goes to brake (then 120 s run-on) or is switched off (20 s failsafe defaultPull → soft brake, then 120 s run-on, ≈ 140 s in total). Decision proposal: keep it this way. While AutoStop holds the drum the VESC drives 18 A brake current into the motor, so cooling is useful. Detecting AutoStop in the receiver (tachometer/duty cycle over UART) would depend on WINCH-16 and would be a change to safety-relevant code; not planned. Etienne's observation after switching the remote off: OLED shows "P 1" with -20 kg, then "B -1". Interpretation (not verified): the failsafe sets state 1, but the receiver's own autostop taper (`tachometer < 10` → `hardBrake = -20`) overrides the pull value; after 20 s the failsafe goes to soft brake. If correct, the UART tachometer is not stuck at 0 any more (relevant to WINCH-16).
- 2026-10-02: confirmed by Etienne: transmitter and receiver still run the **old** firmware (pre WINCH-07). This explains the fan switching on at connect. The WINCH-07 logic has not been flashed or bench-tested yet.
- 2026-10-03: new firmware flashed on both boards. Bench (desk, no motor), Etienne: TX and RX connect, LoRa works, buttons work, fan starts and stops as intended. Still open: brake ↔ pull within 120 s, failsafe with the remote switched off.

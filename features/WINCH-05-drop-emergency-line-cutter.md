# WINCH-05: Drop emergency line cutter

**Status:** Bench Test
**Type:** FW / Doc
**Priority:** P1
**Safety-relevant:** Yes (changes the LoRa struct and the 3rd-button behaviour, which includes a forced hard brake)
**Depends on:** WINCH-02, WINCH-03
**Created:** 2026-09-28 · **Last updated:** 2026-10-02

## Motivation
Bernd's line cutter was physically mounted in the new frame (winter 24/25) but never wired, and there is no plan to do so. The servo code, the 3rd-button long/double click and the `servo` field in the LoRa message are dead weight.

## Scope
- **Transmitter:** remove the `servo` variable, the 3rd-button long click (servo + hard brake) and double click (servo reset), and the `servo` field in `LoraTxMessage`.
- **Receiver:** remove the ESP32Servo include, the `Servo` object, `servoPin` 15 and its handling, the `servo` field in `LoraTxMessage`, and the "Line Cutter" OLED line.
- README.md: remove the line cutter sections, the IO15 pin description, ESP32Servo from the library list, the old "Done ToDo" mention.
- Bundle with WINCH-07 into **one** struct change and one flash of TX + RX.

## Out of scope
- Physically removing the cutter from the frame (Etienne's call).
- A replacement emergency function. Hard brake remains available via the normal buttons.

## Changes
- **Firmware transmitter / receiver:** see scope. `LoraTxMessage` must stay byte-identical in both sketches.
- **Docs:** README.md.

## Before flashing (old firmware still on both boards)

Do these two steps first, with the winch **not in use** (no pilot, line not under tension). Plugging USB into the receiver resets the ESP32. The PPM signal stops, and after 1 s the VESC brakes with its timeout brake current (10 A).

### 1. Back up the current firmware of both boards (esptool)
This is a full image of each board's flash memory, i.e. the old, field-proven state. If the new version misbehaves, both boards can be put back exactly as they were.

esptool comes with the esp32 core of the Arduino IDE:
- Path (Arduino IDE 2, Windows): `%LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\<version>\esptool.exe`. Look in the `esptool_py` folder for the version number.
- If the path is unclear: in the Arduino IDE enable *File → Preferences → Show verbose output during: upload*, then upload any sketch once. The log shows the full esptool path. Do this with a spare board, not with the winch boards.
- COM port: Arduino IDE *Tools → Port*, or Windows Device Manager → "Ports (COM & LPT)". Plug in one board at a time so the port is unambiguous.

PowerShell, one board at a time (replace `<version>`, `COM5` and the date):
```powershell
$esptool = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\<version>\esptool.exe"
mkdir "$env:USERPROFILE\Documents\winch-firmware-backups" -Force
cd "$env:USERPROFILE\Documents\winch-firmware-backups"

# 1. Check the connection and the flash size (expected: 4MB)
& $esptool --chip esp32 --port COM5 flash_id

# 2. Read the whole flash (4 MB = 0x400000; use 0x800000 if flash_id reports 8MB)
& $esptool --chip esp32 --port COM5 --baud 460800 read_flash 0 0x400000 receiver_2026-10-xx.bin

# 3. Check that the file matches the board
& $esptool --chip esp32 --port COM5 --baud 460800 verify_flash 0 receiver_2026-10-xx.bin
```
Repeat with the transmitter (`transmitter_2026-10-xx.bin`). Each file is about 4 MB.
- Keep the backups **outside the repo** (e.g. the folder above, plus a copy on a USB stick or in the cloud). They are full flash images; do not commit them.
- If esptool can't connect: close the Arduino IDE serial monitor (it blocks the port), try `--baud 115200`, check the USB cable (some cables only charge).

**Restore** (only if the new version has to be rolled back), per board:
```powershell
& $esptool --chip esp32 --port COM5 --baud 460800 write_flash 0 receiver_2026-10-xx.bin
```
⚠ Always restore **both** boards: the old and the new firmware use different LoRa packet sizes and do not work together.

### 2. Check the wiring with the old firmware (baseline)
The fan relay doesn't switch (WINCH-07) and the telemetry freezes (WINCH-16). Both could be wiring problems. Check the wiring while the old firmware is still running, so a later fault is clearly due to either the hardware or the new code.

**Receiver, expected wiring according to the code** (identical in the old and new version, except IO15):

| ESP32 pin | Function in code | Should go to | Found (wire colour, OK?) |
|-----------|------------------|--------------|--------------------------|
| IO13 | PPM output | VESC "Servo"/PPM port, signal | |
| GND | PPM ground | VESC "Servo"/PPM port, GND | |
| IO14 | UART RX (`VESC_RX`) | VESC COMM **TX** | |
| IO2 | UART TX (`VESC_TX`) | VESC COMM **RX** | |
| GND | UART ground | VESC COMM GND | |
| IO12 | Relay signal | Relay module IN (white wire) | |
| 5V | Relay supply | Relay module VCC (red wire) | |
| GND | Relay ground | Relay module GND (black wire) | |
| IO15 | Line cutter servo (old firmware only) | nothing (never wired) | |
| ? | Receiver power supply | ? | |

The LoRa module and the OLED are wired on the board itself (no external wires). According to the pinout image in `doc/`, the board's LoRa reset is **GPIO23**. The code uses `RST 14`, i.e. the UART RX pin (see WINCH-16).

**Transmitter:** IO15 → UP button → GND, IO12 → DOWN button → GND, IO14 → 3rd button → GND (old firmware only).

Checks on the receiver:
- [x] **No microSD card in the board's card slot.** The slot uses IO13, IO14, IO15 and IO2, the same pins as the PPM output and the UART. → Confirmed, no card, none planned (Etienne, 2026-10-02).
- [ ] Fill in the table above. Take a photo of the receiver wiring.
- [ ] **UART, power off, multimeter in continuity mode:** IO14 ↔ VESC COMM TX, IO2 ↔ VESC COMM RX, GND ↔ VESC COMM GND. TX and RX must be **crossed** (TX to RX). Check crimps and that the JST plug is fully seated.
- [ ] **UART, running:** turn the drum with the potentiometer, compare `Tac` in VESC Tool (Realtime Data) with the line length on the transmitter OLED. Gently wiggle the UART wires and plugs: if the transmitter display changes or unfreezes, it's a contact problem.
- [ ] **Relay:** with the old firmware, a short press on the 3rd button of the remote toggles the relay. Use this to work through the relay checklist in WINCH-07 (voltage at IO12 against GND, module LED, relay click, fan on COM/NO).

Record the results in WINCH-07 (relay) and WINCH-16 (UART).

## Test plan
### Bench
- [ ] TX and RX flashed as a pair; link established (RSSI shown on both OLEDs).
- [ ] All states −2…5 reachable, receiver shows the correct state/kg.
- [ ] Failsafe: switch off TX during pull state → default pull after 1.5 s, soft brake after 20 s.
- [ ] Old TX + new RX (mismatched sizes) do **not** control the winch. Expected: packets ignored. Document the result.
### Field
- [ ] Normal tow session without anomalies.

## Open questions
- [ ] Does the 3rd button stay physically on the remote (unused), or is it removed (WINCH-12)?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Line cutter is not implemented | Experience: not needed, adds complexity; never wired | 2026-09-28 |

## Log
- 2026-09-28: created
- 2026-09-30: implemented together with WINCH-06/07 (one flash). Transmitter: 3rd button (IO14), servo variable, `LineCutter()`/`setServo()` and the `servo` field removed. Receiver: ESP32Servo, servo object/pin 15 and the "Line Cutter" OLED line removed. `LoraTxMessage` is now 3 bytes (was 5); `static_assert`s in both sketches check 3/4 bytes. Additional safety fix: the admin transmitter's (ID 0) startup scan accepted packets `>=` the TX size; with 3 bytes it would have read the receiver's 4-byte ack as a transmitter message (possibly a wrong start state). Changed to `==`. README, CLAUDE.md updated. Both sketches compile (arduino-cli, esp32 core 2.0.15, board TTGO LoRa32-OLED). Not flashed, not bench-tested.
- 2026-10-02: added "Before flashing": esptool backup/restore of both boards and a wiring check with the old firmware (Etienne's call: rule out wiring problems for the relay and the UART first).

# CLAUDE.md

Working basis for developing this repository together with Claude.

## ⚠ Criticality

This repo is **not** a toy project. It documents and runs a **real, actively used electric paraglider / hang-glider winch** (roughly €6–8k in hardware). Pilots are towed into the air with this code. **Lives depend on this code working correctly.**

- Any change to pull/brake values, failsafe timing, autostop, the PPM mapping or the LoRa protocol needs **explicit confirmation from Etienne** first. Flag it as safety-relevant in its feature file.
- Claude can't bench-test or field-test anything. Never write that something is "tested" or "works". At most it "compiles", and only if that was actually checked. Bench and field tests are done by Etienne and recorded in the feature file.
- Prefer small, readable, conservative changes over clever refactors.

## Project overview

Remote-controlled electric winch for paragliders and hang gliders (step towing / self-launch). Forked from Robert Zach's [ewinch_remote_controller](https://github.com/robertzach/ewinch_remote_controller).

### Hardware chain

```
Transmitter (handheld)                Receiver (on winch)                    Winch
TTGO LoRa32 V2.1_1.6  --LoRa 868MHz-->  TTGO LoRa32 V2.1_1.6  --PPM IO13-->   Trampa VESC 75/300  -->  QS Motor 12kW 260 V4 (hub motor)
2 buttons (+ optional 3rd)            <--ack (pull, tacho,   <--UART IO14 RX/    (patched firmware)        drum + barrel-cam winding
OLED                                     battery, temp)          IO2 TX--                                  azimuth system (Bernd)
                                                              IO12 relay -> VESC cooling fan
```

- **Battery:** 16S10P, 2× 8S10P packs in series (Samsung INR21700-40T), ~57.6 V nominal, 67.2 V max. 8S packs so they can be charged with standard RC chargers (2× ISDT AIR8).
- **Frame:** bike trailer, winding via barrel-cam mechanism, line guided by the "azimuth system" (designed by Bernd Otterpohl, built by Nico & Hans Werner Stucke). The earlier rotating mount with arm is outdated.
- Parts and sources: [doc/parts-list.md](doc/parts-list.md). Overview diagram: [doc/winch-schema.jpg](doc/winch-schema.jpg).

## Repository layout

| Path | Content |
|------|---------|
| `transmitter/` | Handheld remote sketch. **`transmitter_with-monitor-support.cpp` is the in-use version** (synced in WINCH-02; ID 3, `myMaxPull = 95`, with ESP-NOW monitor code). `transmitter.ino` is an older variant without monitor support. File naming / cleanup in WINCH-03/04 |
| `receiver/` | Winch-side sketch (`receiver.ino`) + helper module `LiPoCheck.cpp/.h` (battery % from cell voltage) |
| `vesc/` | Patched VESC firmware binaries, VESC app/motor configs (XML), `vesc_ppm_auto_stop.patch` (reference only) |
| `doc/` | Parts list, DXF/STL files, photos, motor manuals |
| `monitor-LoRa/`, `monitor-ESP-NOW/` | Cockpit monitor, **being retired** → moves to `old/` (WINCH-04) |
| `features/` | Roadmap and feature tracking ([features/INDEX.md](features/INDEX.md)) |
| `old/` | (planned) archived code/docs that are no longer active |

## Toolchain: Arduino IDE only

- **No PlatformIO.** Etienne opens or copies the sketch into the Arduino IDE, compiles it and flashes the ESP32 board.
- Board package: "esp32 by Espressif Systems". Board: **TTGO LoRa32-OLED**.
- Libraries: LoRa (sandeepmistry), Button2, VescUart (SolidGeek), ESP8266-OLED-SSD1306 (ThingPulse), Pangodream 18650CL. (ESP32Servo is only needed while the line cutter code exists; it is being removed in WINCH-05.)
- **File format rule:** every sketch folder contains exactly **one** main file named `<folder>.ino`. The Arduino IDE compiles *all* `.cpp`/`.ino` files in a sketch folder together, so a second main file breaks the build. `.cpp` main files are PlatformIO leftovers: keep the newer version, delete the older one, rename to `.ino`. Real helper modules such as `LiPoCheck.cpp/.h` stay `.cpp/.h`.
- Keep code copy-paste friendly: no build flags, no custom partition tables, no extra tooling.

## VESC firmware is frozen

- The controller is a **Trampa VESC 75/300** (HW `75_300_R3`). It runs the patched firmware [vesc/vesc_75_300_auto_stop.bin](vesc/vesc_75_300_auto_stop.bin) (committed by Robert Zach, 2022-07-08, most likely **FW 5.x**; the exact flashed version is still to be read out in VESC Tool, see WINCH-08).
- **Do not touch, rebuild or propose changes to the VESC firmware, the patch or the VESC configs** unless Etienne explicitly asks for it. `vesc_ppm_auto_stop.patch` is reference documentation for understanding autostop behaviour only.
- A 6.02 build (`VESC_75_300_AutoStop.bin`) existed in the repo briefly in May 2024 and was deleted. It is not the reference firmware.

## Architecture & protocol

> ⚠ **Changes in this section need extra attention: lives depend on it.** Always confirm with Etienne, and mark the feature file as safety-relevant. TX and RX must always be flashed as a matching pair.

### Pull state machine (transmitter)

| State | Name | Target pull (transmitter.ino) |
|------:|------|-------------------------------|
| -2 | hard brake | `hardBrake = -20` kg |
| -1 | soft brake | `softBrake = -7` kg (start state) |
| 0 | neutral (no pull, no brake) | 0, reachable only from brake via double click on DOWN |
| 1 | default pull | `defaultPull = 7` kg |
| 2 | pre pull | `prePullScale = 18` % of `myMaxPull` (≈17 kg) |
| 3 | take-off pull | `takeOffPullScale = 55` % |
| 4 | full pull | `fullPullScale = 80` % |
| 5 | strong pull | `strongPullScale = 100` % |

`myMaxPull = 95` (0–127 "kg", scaled via VESC PPM/current settings; roughly 3.6 A/kg on this motor). Etienne sets it to roughly his take-off weight. With 95: state 2 = 17, 3 = 52, 4 = 76, 5 = 95 kg (integer math). The ESP-NOW monitor can overwrite `myMaxPull` at runtime (no range check).

Buttons: UP (IO15) moves one state up (at most once per second, skips neutral). DOWN (IO12) goes from any pull >1 back to default pull (1), or from 0/-1 one step down; a short press in state 1 does nothing. DOWN long press (500 ms) → soft brake. DOWN double click from brake → neutral. The 3rd button (IO14) currently toggles the relay (short), fires the line cutter (long) and resets it (double). It is being removed in WINCH-05/07.

The receiver keeps its **own** copies of some values (`softBrake = -8`, `defaultPull = 8`, scale values differing slightly). It uses them for failsafe and autostop, not for the normal pull states, which come from the transmitter.

### LoRa link

- 868 MHz, TX power 20 dBm, CRC enabled.
- `LoraTxMessage` (TX→RX): `id:4`, `currentState:4`, `pullValue`, `pullValueBackup`, `servo`, `relay`. `LoraRxMessage` (RX→TX ack): `pullValue`, `tachometer` (×10 m), `dutyCycleNow`, battery/motor-temp alternating in 1+7 bits.
- **Both structs must be byte-identical in both sketches.** Packets are matched by `sizeof`, so any struct change requires flashing transmitter and receiver together.
- Transmitter sends every 400 ms and immediately on a state change. The receiver acks every valid packet.
- ID lock: the receiver only follows one transmitter ID. A different ID can take over after 5 s of silence. Admin ID 0 can always take over.

### Receiver behaviour

- **Failsafe** (only if state ≥1): no packet for >1.5 s → default pull. After 20 s without a packet → soft brake. (A code comment says "10 seconds"; the code uses 20 s.)
- **Smoothing:** pull increases at max ~65 kg/s and decreases at ~90 kg/s. Brake values (<0) apply immediately.
- **PPM output** on IO13: `(currentPull + 127) * (2000 − 950) / 254 + 950` µs, one pulse per loop (~20 ms).
- UART to VESC every 20 loops: battery %, motor temp, tachometer, duty cycle.

## Autostop

Autostop exists on two layers:

1. **VESC firmware (patch), according to `vesc_ppm_auto_stop.patch`:**
   - In normal PPM mode, autostop is always active: tachometer < 1500 (≈15 m line left) → 18 A brake current.
   - Potentiometer on **ADC2**: above 0.5 V the VESC switches to *manual current mode* (0–35 A, proportional to the poti) and **ignores the PPM input from the receiver**. That is why the poti must be fully at its "off" end during normal operation.
   - In manual mode, autostop is only active if **ADC1 > 3 V**. ⚠ *Unverified:* this is only in the patch source; it is not confirmed that the `.bin` was built from exactly this patch, nor how ADC1 is wired on Etienne's winch (WINCH-08).
2. **Receiver sketch:** a taper for tachometer values 2–40 (limit to default pull, then soft brake, then hard brake). Its tachometer units do not obviously match the patch's (1500 = 15 m), so this is a known open point.

### ⚠ Rewind rule (safety)

**After releasing, rewind the line with low pull only: maximum state 2 (prePull).** With more pull the autostop brake cannot stop the drum in time: the carabiner gets pulled into the azimuth system and destroys it, or the line snaps. **This has already happened once**, and the whole winch had to be rebuilt. The old README text ("release and move back to fullPull to rewind") was wrong and is corrected in WINCH-01.

## Known quirks / tech debt

Found while reading the code. Not fixed yet. The repo may also be behind Etienne's local versions (WINCH-02); review in WINCH-15.

- `#define RST 14` (LoRa reset) collides with `VESC_RX 14` on the receiver and `BUTTON_THREE 14` on the transmitter. On the TTGO LoRa32 V2.1_1.6 the LoRa reset is probably GPIO23.
- IO12 is an ESP32 strapping pin (flash voltage), used for the relay (receiver) and BUTTON_DOWN (transmitter).
- `LoRa.begin(868E6)` is hardcoded, so the `BAND` define is unused.
- Failsafe comment says 10 s, the code uses 20 s.
- Receiver autostop tachometer thresholds (2–40) vs. the patch (1500 ≈ 15 m) use different units.
- *Etienne's observation:* the reported line length is about 0.7× the real length, although the drum diameter and related settings in the VESC app appear to be entered correctly.
- `transmitter_with-monitor-support.cpp` (in-use version, includes ESP-NOW monitor code) sits in the same sketch folder as `transmitter.ino` (older, 2024-02-16), so the folder does not compile as-is (WINCH-03).
- Transmitter: if ESP-NOW init or add-peer fails, `setup()` returns before the OLED and button handlers are set up.

## Workflow

- Every hardware, firmware or documentation change is tracked as a feature: `features/WINCH-NN-<slug>.md` + a row in [features/INDEX.md](features/INDEX.md). Update the status as work progresses.
- Status flow: **Idea → Planned → In Progress → Bench Test → Field Test → Done**, or **Dropped**.
- Bench and field test results are recorded by Etienne (or with him) in the feature's test plan / log.
- Commit only when Etienne asks. The GitHub repo is **public** (`looxis-renaud/remotecontrolled_electric_winch`), so never commit secrets or personal data.
- All repo documentation is written in **English**.

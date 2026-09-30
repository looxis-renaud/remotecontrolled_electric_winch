# WINCH-04: Archive cockpit monitor → `old/`

**Status:** Bench Test
**Type:** Doc / FW
**Priority:** P0
**Safety-relevant:** No (transmitter change only comments out ESP-NOW; pull/brake, failsafe, LoRa protocol untouched)
**Depends on:** WINCH-03
**Created:** 2026-09-28 · **Last updated:** 2026-09-30

## Motivation
The cockpit monitor (LilyGO T-Display S3, via LoRa or ESP-NOW) was lost in flight once, and experience showed it adds complexity without being needed. It won't be rebuilt.

## Scope
- Move into `old/` (with `git mv`, so history is kept):
  - `monitor-LoRa/`
  - `monitor-ESP-NOW/`
  - `doc/t-display-s3-pinout.jpg`
- ~~the monitor-support transmitter variant~~ → since WINCH-03 this is the active `transmitter/transmitter.ino`. Instead, its ESP-NOW code is **commented out** (not deleted), see Decision log.
- Add a short `old/README.md`: what is in there and why it was retired.
- README.md: remove the sections "Monitor with LilyGo T-Display" and "Pin Setup Monitor", the monitor mention in the frequency note, and TFT_eSPI from the library list.
- `doc/parts-list.md`: remove the "Monitor" section.

## Out of scope
- Line cutter / warning light / fan text → WINCH-05/06/07.
- Deleting the commented-out ESP-NOW code.

## Changes
- **Firmware transmitter:** ESP-NOW monitor code commented out in four blocks marked `[WINCH-04]`:
  1. `#include <esp_now.h>`, `#include <WiFi.h>`, monitor MAC
  2. `EspNowTxMessage` / `EspNowButtonMessage` structs, `peerInfo`, `OnDataSent` / `OnDataRecv` callbacks
  3. `setup()`: WiFi / ESP-NOW init, peer registration, callbacks
  4. `loop()`: sending values to the monitor
  - Checked mechanically (comments stripped, active code compared to the previous commit): the only removed active code is ESP-NOW; nothing else changed; no stray comment markers.
  - Behaviour effects once flashed: WiFi is no longer started; `myMaxPull`, `relay` and `servo` can no longer be changed by a monitor; `setup()` can no longer return early on an ESP-NOW error (known issue from WINCH-02 is gone).
- **Docs:** README.md, doc/parts-list.md, CLAUDE.md, new `old/README.md`.

## Test plan
- [x] `grep -ri monitor` on README.md / parts-list only finds intentional mentions (pointer to `old/`).
- [ ] `transmitter/transmitter.ino` compiles in the Arduino IDE (Etienne).
- [ ] After flashing: remote starts normally (OLED, buttons, LoRa link to receiver, all pull states) (Etienne, bench).

## Open questions
- [x] Anything else in `doc/` only relevant for the monitor? → No. `5442780104968238496.jpg` is a line pulley, `photo1665499714.jpeg` is the LoRa32 board.
- [ ] Flash now, or together with WINCH-05/06/07 (which change the transmitter again)?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Retire the monitor, archive instead of delete | Lost in flight, extra complexity; keep code for reference | 2026-09-28 |
| Comment out ESP-NOW code in `transmitter.ino` instead of deleting it | Etienne wants it disabled but kept in place | 2026-09-28 |

## Log
- 2026-09-28: created
- 2026-09-28: monitor folders + T-Display pinout moved to `old/`, `old/README.md` added, ESP-NOW code in transmitter commented out, README / parts list / CLAUDE.md updated. Not compiled.
- 2026-09-30: set to Bench Test (Etienne). Repo work complete. Open: compile in the Arduino IDE, flash the remote, bench check (see Test plan). The remote still runs the old firmware with ESP-NOW until reflashed.

# WINCH-20: Pin toolchain & store libraries in the repo

**Status:** In Progress
**Type:** FW / Doc
**Priority:** P1
**Safety-relevant:** Yes (the flashed firmware depends on core and library versions; e.g. Button2 drives the pull state machine)
**Depends on:** –
**Created:** 2026-09-30 · **Last updated:** 2026-09-30

## Motivation
Etienne has used two Arduino IDE versions over time (one worked, one didn't), and nothing was updated for over a year. Newer versions of the esp32 core (3.x) and of libraries such as Button2 exist. For a safety-critical system a reproducible build matters more than the newest version: the firmware must always be rebuildable with the same code.

## Scope
- Record the known-good toolchain: Arduino IDE 2.3.2, esp32 core 2.0.15, board TTGO LoRa32-OLED.
- Store unmodified copies of the five libraries in `arduino-libs/` (with license files) and document versions, sources and usage in `arduino-libs/README.md`.
- README install section and CLAUDE.md point to the pinned versions.
- Rule: update core or libraries only one at a time, as a separate feature, with a bench test.
- Remove the old Arduino IDE 1.8.x (done by Etienne, 2026-09-30).

## Out of scope
- Updating to newer core or library versions (each would be its own feature).
- Storing the esp32 core in the repo (too large; pinned by version instead).

## Changes
- **Repo:** new `arduino-libs/` (Button2 2.3.2, LoRa 0.8.0, VescUart 1.0.1, ESP8266 and ESP32 OLED driver for SSD1306 displays 4.5.0, Pangodream_18650_CL 1.0.1; about 1.1 MB, MIT / GPL-3.0).
- **Docs:** `arduino-libs/README.md`, README.md, CLAUDE.md.

## Test plan
### Bench (workshop, no pilot)
- [x] Both sketches compile against `arduino-libs/` (arduino-cli, `--libraries arduino-libs`, 2026-09-30). Same program size as with the IDE's installed libraries.
- [x] Old Arduino IDE 1.8.x removed (Etienne, 2026-09-30).
- [x] In Arduino IDE 2.3.2: board TTGO LoRa32-OLED selectable, the libraries show as installed (Etienne, 2026-09-30). Installed versions = the copies in `arduino-libs/` (same folder, checked with `arduino-cli lib list`); esp32 core 2.0.15 is the only esp32 core installed.
- [ ] Firmware built with these versions flashed and bench-tested (together with WINCH-05/06/07).

## Open questions
- [ ] Was the firmware currently on the winch built with these versions? The libraries on this machine were installed February to April 2024; the transmitter code was last changed on another machine (WINCH-02).

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Keep known-good versions, update only one at a time with bench test | Reproducible build for a safety-critical system; esp32 core 3.x has breaking changes | 2026-09-30 |
| Store library copies in the repo (`arduino-libs/`) | Libraries can change or disappear upstream; licenses (MIT, GPL-3.0) allow redistribution with license file | 2026-09-30 |

## Log
- 2026-09-30: created. Libraries copied from `Documents\Arduino\libraries`, compile check against the copies passed. Docs updated.
- 2026-09-30: old Arduino IDE 1.8.x removed; in IDE 2.3.2 the board is selectable and the libraries show as installed (Etienne). Remaining: flash + bench test with WINCH-05/06/07.

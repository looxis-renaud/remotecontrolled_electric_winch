# WINCH-03: Source-file cleanup & drop PlatformIO

**Status:** Planned
**Type:** FW / Doc
**Priority:** P0
**Safety-relevant:** No (file organisation only, no logic changes)
**Depends on:** WINCH-02
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
The move to VS Code + PlatformIO was never completed and won't be. Etienne works with the Arduino IDE: open the sketch, compile, flash. PlatformIO left `.cpp` main files behind, which the Arduino IDE compiles *together* with the `.ino` in the same sketch folder. Two main files mean duplicate `setup()`/`loop()`, and the build fails.

## Scope
- Rule: each sketch folder contains exactly one main file `<folder>.ino`.
- Where an `.ino` and a `.cpp` main file coexist: keep the newer / in-use one (per WINCH-02), delete the older one, rename to `.ino`.
  - `transmitter/`: `transmitter.ino` (2024-02-16) vs `transmitter_with-monitor-support.cpp` (2024-04-16, contains ESP-NOW monitor code). Because the monitor is being retired (WINCH-04), the newer file probably moves to `old/` rather than replacing `transmitter.ino`. Decide together.
- Helper modules stay: `receiver/LiPoCheck.cpp/.h`.
- Remove all PlatformIO references from README.md (section "Moving to Visual Studio Code and PlatformIO").
- Update the library list in README (remove TFT_eSPI together with WINCH-04; ESP32Servo with WINCH-05).

## Out of scope
- Logic changes.
- Monitor folders themselves → WINCH-04.

## Changes
- **Firmware:** file renames/deletions only.
- **Docs:** README.md toolchain section.

## Test plan
- [ ] `transmitter/` and `receiver/` each open and compile in the Arduino IDE without errors (Etienne).
- [ ] `grep -ri platformio` finds nothing outside `old/` and `features/`.

## Open questions
- [ ] Which transmitter variant becomes `transmitter.ino` (see WINCH-02)?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Arduino IDE only, no PlatformIO | Simpler for Etienne: open, compile, flash | 2026-09-28 |

## Log
- 2026-09-28: created

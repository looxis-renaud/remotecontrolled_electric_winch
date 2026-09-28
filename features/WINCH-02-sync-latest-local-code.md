# WINCH-02: Sync repo with latest local code

**Status:** Planned
**Type:** FW
**Priority:** P0
**Safety-relevant:** Yes (the flashed code is what pilots rely on)
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
The repo code is about two years old (last sketch changes: `receiver.ino` 2024-04-04, `transmitter.ino` 2024-02-16, `transmitter_with-monitor-support.cpp` 2024-04-16). Etienne may have newer versions with small changes on another machine. Those may be exactly what is flashed on the winch today. All later code features must build on the code that is actually in use.

## Scope
- Collect the transmitter and receiver sketches from Etienne's other machine.
- Diff them against the repo versions (`transmitter/transmitter.ino`, `transmitter/transmitter_with-monitor-support.cpp`, `receiver/receiver.ino`, `receiver/LiPoCheck.*`).
- Determine which variant is actually flashed on the handheld and on the receiver (the OLED start screen and behaviour can help, e.g. does the transmitter try ESP-NOW?).
- Commit the newest/in-use versions as the new baseline, with a commit message listing the differences.

## Out of scope
- Cleaning up file names / PlatformIO → WINCH-03.
- Functional changes.

## Changes
- **Firmware transmitter / receiver:** replace with the in-use versions (no logic changes by Claude).

## Test plan
- [ ] Diff reviewed together; every difference explained.
- [ ] Baseline compiles in the Arduino IDE (Etienne).

## Open questions
- [ ] Where are the newer files (machine, path)?
- [ ] Which transmitter variant is flashed: with or without monitor support?
- [ ] Is a VESC config backup from the actual controller available (to compare with `vesc/240601_*.xml`)?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-09-28: created

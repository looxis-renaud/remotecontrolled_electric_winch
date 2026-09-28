# WINCH-04: Archive cockpit monitor → `old/`

**Status:** Planned
**Type:** Doc / FW
**Priority:** P0
**Safety-relevant:** No
**Depends on:** WINCH-03
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
The cockpit monitor (LilyGO T-Display S3, via LoRa or ESP-NOW) was lost in flight once, and experience showed it adds complexity without being needed. It won't be rebuilt.

## Scope
- Move into `old/` (with `git mv`, so history is kept):
  - `monitor-LoRa/`
  - `monitor-ESP-NOW/`
  - the monitor-support transmitter variant (`transmitter/transmitter_with-monitor-support.cpp`), unless WINCH-02/03 decide otherwise
  - `doc/t-display-s3-pinout.jpg`
- Add a short `old/README.md`: what is in there and why it was retired.
- README.md: remove the sections "Monitor with LilyGo T-Display" and "Pin Setup Monitor", the monitor mention in the frequency note, and TFT_eSPI from the library list.
- `doc/parts-list.md`: remove the "Monitor" section.

## Out of scope
- Line cutter / warning light / fan text → WINCH-05/06/07.

## Changes
- **Firmware:** none on the active sketches (only moves).
- **Docs:** README.md, doc/parts-list.md, new `old/README.md`.

## Test plan
- [ ] `grep -ri monitor` on README.md / parts-list only finds intentional mentions (e.g. a pointer to `old/`).
- [ ] Active sketches still compile (Etienne).

## Open questions
- [ ] Anything else in `doc/` only relevant for the monitor?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Retire the monitor, archive instead of delete | Lost in flight, extra complexity; keep code for reference | 2026-09-28 |

## Log
- 2026-09-28: created

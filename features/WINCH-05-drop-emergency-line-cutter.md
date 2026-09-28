# WINCH-05: Drop emergency line cutter

**Status:** Planned
**Type:** FW / Doc
**Priority:** P1
**Safety-relevant:** Yes (changes the LoRa struct and the 3rd-button behaviour, which includes a forced hard brake)
**Depends on:** WINCH-02, WINCH-03
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

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

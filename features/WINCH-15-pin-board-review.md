# WINCH-15: Pin / board review (tech-debt list)

**Status:** Idea
**Type:** FW
**Priority:** P2
**Safety-relevant:** Yes
**Depends on:** WINCH-02 (review the code that is actually in use)
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
While reading the code for CLAUDE.md, several questionable spots came up. They work in practice today, but should be checked deliberately instead of being "fixed" by accident:

- `#define RST 14` (LoRa reset) collides with `VESC_RX 14` (receiver) and `BUTTON_THREE 14` (transmitter). On the TTGO LoRa32 V2.1_1.6 the LoRa reset is probably GPIO23.
- IO12 is an ESP32 strapping pin (flash voltage). It is used for the relay (receiver) and BUTTON_DOWN (transmitter). An external pull-up at boot can prevent booting.
- `LoRa.begin(868E6)` is hardcoded; the `BAND` define is unused.
- Failsafe comment says 10 s; the code uses 20 s. Decide which is intended and align the comment (not the timing) unless Etienne decides otherwise.
- Receiver autostop tachometer thresholds (2–40) vs. the VESC patch (1500 ≈ 15 m): different units, and the receiver taper may never trigger.
- Line length reported ~0.7× too short (Etienne's observation; VESC settings seem correct).

## Approach
Check each item against the board pinout and real behaviour, then decide per item: leave it and document it, fix the comment, or fix the code (as its own safety-relevant change with a bench test).

## Log
- 2026-09-28: created as idea
- 2026-10-03: receiver supply documented (README "Receiver wiring harness"): the VESC powers the receiver through the board's small 2-pin **battery** connector (on the transmitter it goes to an 18650 cell). Open: which voltage the VESC delivers there. That input is designed for a single Li-ion cell (about 3.0–4.2 V) and feeds the onboard charger; if it gets 5 V, check that this is within the board's limits or move the supply to the 5V pin.
- 2026-10-03: **LoRa reset pin fixed in both sketches: `RST 14` → `RST 23`** (Etienne asked to review and correct the code before reflashing). The board pinout (doc/LilyGO-…-pinout….jpg, T3_V1.6.1) shows LoRa RST = GPIO23. On the receiver, IO14 is also `VESC_RX`, and `LoRa.begin()` set it as an output (HIGH) after `Serial1.begin()`. On the transmitter IO14 is unused since WINCH-05. Safety-relevant (LoRa init): bench test on the desk before using it at the winch. Compiles (arduino-cli, esp32 2.0.15); not flashed, not tested.
- 2026-10-03: unused `myMaxPull = 85` and the four `…PullScale` variables removed from `receiver.ino` (never referenced; the pull states are computed in the transmitter only). No behaviour change. Compiles.
- 2026-10-03: bench: with `RST 23` both boards start, LoRa link works (Etienne). The fix is confirmed on the bench.

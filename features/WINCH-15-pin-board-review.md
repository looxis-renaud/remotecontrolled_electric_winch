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

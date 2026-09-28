# WINCH-08: Autostop documentation & verification

**Status:** Planned
**Type:** Doc / VESC (read-only)
**Priority:** P0
**Safety-relevant:** Yes
**Depends on:** WINCH-01
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
Autostop protects the azimuth system and the line at the end of the rewind, but only under the right conditions. The README doesn't explain how the potentiometer is wired, which poti position does what, or the limits of the brake. Some of what we "know" comes only from the patch source and isn't verified on the real winch.

## Scope
- Verify on the real winch (VESC Tool, multimeter), **read-only; no firmware or config changes**:
  - [ ] actually flashed firmware version (VESC Tool → Firmware), expected 5.x patched
  - [ ] poti wiring: which pins go to 3.3 V / GND / ADC2 on the COMM port
  - [ ] ADC1: connected? To what? Does ADC1 > 3 V really enable autostop in manual mode (patch line 34)?
  - [ ] voltage at ADC2 with poti fully at the "off" end (must be < 0.5 V)
  - [ ] direction: which way is "off" (Etienne's notes say "fully left", to be confirmed)
- Write a README section "Autostop & potentiometer":
  - wiring diagram / pin table
  - poti positions ↔ function (off/normal PPM operation vs. manual rewind with proportional current 0–35 A)
  - what autostop does (≈15 m line left → 18 A brake) and what the receiver-side taper does
  - ⚠ the rewind rule (max. state 2, from WINCH-01) repeated here
  - recovery procedure: line on the ground / remote switched off too early → gently turn the poti to rewind
- Document open points (tachometer units RX vs. patch; Etienne's observation that line length reads ~0.7× too short).

## Out of scope
- Any change to VESC firmware, patch or configs (frozen, see CLAUDE.md).
- Fixing the line-length offset.

## Changes
- **VESC:** none (read-only verification).
- **Docs:** README.md, possibly `vesc/readme.md`.

## Test plan
### Bench
- [ ] Poti at off end → winch follows the remote normally.
- [ ] Poti turned slightly → winch pulls gently regardless of the remote (manual mode).
- [ ] Autostop triggers at ≈15 m remaining line with a low rewind pull (state ≤2). Measure the remaining distance.

## Open questions
- [ ] All verification items above.
- [ ] Should a photo of the poti wiring go into `doc/`?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Autostop docs are based on verified facts only; unverified items are marked as such | Safety documentation must not guess | 2026-09-28 |

## Log
- 2026-09-28: created

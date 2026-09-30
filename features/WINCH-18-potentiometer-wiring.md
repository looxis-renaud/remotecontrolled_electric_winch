# WINCH-18: Potentiometer wiring & part spec (ADC2 manual mode)

**Status:** In Progress
**Type:** HW / Doc
**Priority:** P1
**Safety-relevant:** Yes (a wrongly wired, floating or broken potentiometer puts the VESC into manual mode: the winch pulls on its own and ignores the remote)
**Depends on:** –
**Created:** 2026-09-30 · **Last updated:** 2026-09-30

## Motivation
Several places in the docs say a potentiometer must be connected to the VESC's ADC input (ADC2, see [vesc/readme.md](../vesc/readme.md), "Line auto stop in VESC"), and that ADC2 must be connected to GND if no potentiometer is fitted. Without a potentiometer or without the ADC2–GND connection, the winch simply starts pulling at full speed. So far the docs did not say how the potentiometer is wired or what kind of potentiometer to use.

## Scope
- Document the wiring (pin ↔ COMM signal ↔ wire colour) in `vesc/readme.md`.
- Recommend a potentiometer type (resistance, taper, mechanics).
- Document the check after wiring and the failure modes (floating ADC2, broken GND/VCC/wiper wire).

## Out of scope
- VESC firmware, patch, configs (frozen).
- Full autostop documentation and verification: WINCH-08. WINCH-08's open item "poti wiring" is covered by the wiring table here; the remaining WINCH-08 checks (ADC1, direction, voltage at the off end) stay there.

## Changes
- **Hardware:** none so far (documents the existing wiring).
- **VESC:** none (firmware is frozen, see CLAUDE.md)
- **Docs:** `vesc/readme.md`, section "Line auto stop in VESC": subsections "Potentiometer: wiring", "Potentiometer: which part", "Potentiometer: failure modes".

## Wiring (Etienne, 2026-09-30)
Potentiometer viewed from below:

| Potentiometer pin | VESC COMM signal | Wire colour (JST cable) |
|---|---|---|
| left | GND | yellow |
| middle (wiper) | ADC2 | orange |
| right | VCC | red |

## Test plan
### Bench (workshop, no pilot)
- [ ] Voltage on the VCC wire measured: 3.3 V (not 5 V). Value: ___
- [ ] Potentiometer at the "off" end: ADC2 ≈ 0 V (VESC Tool or multimeter). Value: ___ Which end (left/right)? ___
- [ ] Turning slowly: voltage rises smoothly to ≈ 3.3 V; manual mode starts above 0.5 V.
- [ ] Back at the "off" end: winch follows the remote normally.

### Field (real towing)
- [ ] Not needed beyond the WINCH-08 / WINCH-09 pre-flight check (potentiometer at the off end).

## Open questions
- [ ] VCC on the COMM port of this controller: 3.3 V? Pin positions on the COMM port (the Trampa manual shows the newer MKVI revision).
- [ ] Which potentiometer is currently fitted (resistance, taper, part number)?
- [ ] Protect against a broken wiper wire with a pull-down resistor (e.g. 100 kΩ from ADC2 to GND)? This would keep ADC2 near 0 V if the wiper loses contact. It does **not** help against a broken GND wire. Hardware change, needs Etienne's decision.
- [ ] Photo of the wiring for `doc/`?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-09-30: created. Wiring as described by Etienne; wiring, part recommendation, check and failure modes written into vesc/readme.md.

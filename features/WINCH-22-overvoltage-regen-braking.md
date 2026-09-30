# WINCH-22: Over-voltage fault when braking (regenerative braking)

**Status:** Planned
**Type:** HW / VESC (read-only) / Doc
**Priority:** P0
**Safety-relevant:** Yes (all brakes of the winch are regenerative: soft brake, hard brake, autostop. If the VESC shuts down with an over-voltage fault while braking, it stops braking and the drum runs free.)
**Depends on:** –
**Created:** 2026-09-30 · **Last updated:** 2026-09-30

## Motivation
On 2026-09-30 (VESC Tool session, WINCH-17) Etienne turned the potentiometer slowly to full (motor at maximum speed, duty 95 %, drum without line load), then turned it back down quickly. The VESC braked the motor (motor current down to about −60 A) and then reported **`FAULT_CODE_OVER_VOLTAGE`**:
- `faults`: fault at **72.11 V** (= `l_max_vin` 72 V).
- Realtime Data afterwards showed **Volts In 88.0 V** (the controller is rated for spikes up to 75 V).
- At standstill afterwards: 63.1 V (normal). No fault after a restart.

The remote was on, in state −1 (soft brake). In potentiometer mode the VESC ignores the PPM input; when the potentiometer goes back below 0.5 V, normal PPM operation (soft brake from the remote) and the braking take over.

## Suspected cause (not verified)
The braking energy could not flow back into the battery fast enough, so the DC bus voltage rose. Candidates:
- **Daly BMS** (installed August '26): charge path blocked or limited when regen current flows (charge MOSFET off, charge over-current protection, charge current limit). Etienne: charge and discharge can be enabled separately; the VESC and the charger use the same port (common port).
- Before the BMS (balancer only), regen went straight into the cells. Unknown whether over-voltage faults happened before.
- Battery internal resistance and wiring: at about 60 A a rise of only a few volts is expected, not 9+ V, so this alone is unlikely.
- Fast brake transient (VESC ramp settings, battery current limit `l_in_current_min = −290 A`).

## Scope (diagnosis first)
- [ ] Daly app: charge MOSFET state, charge over-current limit / protection settings, alarm/event log at the time of the test.
- [ ] Reproduce carefully at **low** speed first, then step by step higher, with VESC Tool Realtime Data (Volts In, I Batt) recording; stop at the first voltage rise above ~70 V.
- [ ] Check whether the same happens with the remote (hard brake from a pull state at moderate speed, winch on the stand, no pilot).
- [ ] Decide the fix (BMS settings, VESC battery regen current limit `l_in_current_min`, max input voltage, ...). Any VESC config change needs Etienne's decision and a bench test.

## Until resolved
- Don't brake from high drum speed; potentiometer only with little travel (it has no speed limit).
- Towing: the rewind rule (max. state 2) keeps speeds low. Braking from a pull state at speed is exactly the risky case.

## Out of scope
- VESC firmware changes (frozen).

## Test plan
### Bench (workshop, no pilot)
- [ ] Brake from increasing speeds (potentiometer and remote hard brake): Volts In stays below 70 V, no fault.
- [ ] Autostop brake (18 A) from rewind speed: no fault.

### Field (real towing)
- [ ] Normal tow session with brake manoeuvres: no over-voltage fault (`faults` checked afterwards).

## Open questions
- [ ] Did over-voltage faults happen before the BMS was installed?
- [ ] Daly BMS: charge current limit and protection settings?
- [ ] Was the 88 V a real voltage on the DC bus or a measurement artefact after the fault?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| BMS analysis postponed, hall setup first (Etienne) | Hall detection runs at low speed and low current, no high-speed braking | 2026-09-30 |

## Log
- 2026-09-30: created after the over-voltage fault in the VESC Tool session (WINCH-17).

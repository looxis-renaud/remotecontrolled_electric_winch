# WINCH-09: User & maintenance manual

**Status:** Planned
**Type:** Doc
**Priority:** P1
**Safety-relevant:** Yes (operating procedures)
**Depends on:** WINCH-01, WINCH-08
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
Several seasons of towing experience exist only in Etienne's head. A written manual makes operation repeatable and safer for Etienne, winch operators and other builders. The README usage section is short and partly outdated.

## Scope
New `doc/manual.md`, linked from the README. Content gathered in interview sessions with Etienne:
1. **Transport & setup:** trailer, anchoring (ground stakes / straps), line layout, azimuth system alignment.
2. **Pre-flight checklist:** battery charge, remote batteries, LoRa link check, poti at off end, line & weak link, brake test.
3. **Towing procedure:** state sequence (default → pre → take-off → full), step towing, communication, what to do on link loss (failsafe behaviour).
4. **Release & rewind:** ⚠ max. state 2, autostop, what to do if the line lands on the ground.
5. **After the session / packing.**
6. **Battery:** charging (2× ISDT AIR8, 8S packs), storage voltage, balancing, long-term storage.
7. **Maintenance intervals:** line (Dyneema) inspection/replacement, weak link, bearings & pulleys of the azimuth system, barrel-cam winding, connectors (XT90, Anderson), VESC cooling, cable strain relief, remote.
8. **Troubleshooting:** known failure modes and fixes.
9. **Incident log:** what went wrong and what was learned (e.g. the rewind incident; the line wrap after pulling the line out in neutral, which destroyed the line and the 3D-printed gear: pull out with soft brake only, see README).
10. **Several pilots / admin remote:** one remote per pilot (own ID and max pull), red admin remote (ID 0), takeover rules (see README "Several pilots / several remotes").

## Out of scope
- Build instructions (the repo is explicitly not a build guide).

## Changes
- **Docs:** new `doc/manual.md`, README link, and slimming down the README usage section to a pointer.

## Collected content: incident log
Collected here until `doc/manual.md` exists.

| Incident | Cause | Consequence | Lesson / measure |
|----------|-------|-------------|------------------|
| Carabiner pulled into the azimuth system | Line rewound with too much pull after release (old README said fullPull) | Azimuth system destroyed / line snapped, whole winch rebuilt | Rewind with max. state 2 only (WINCH-01, README) |
| Line fell onto the towing track after release | Remote switched off right after release, before the line was fully rewound and AutoStop had stopped the drum. The receiver does not know about AutoStop: its failsafe kept defaultPull for 20 s, then soft brake | Rest of the line lay on the towing track; rewound afterwards with the potentiometer (README E) | Keep the remote on (state 1 or 2) until AutoStop has stopped the drum, only then switch it off (README C, 2026-10-02); checklist item (WINCH-24) |
| Line wrap at the drum on launch | Line pulled out in neutral (state 0) while towing alone; the drum overran when Etienne stopped walking and loose turns formed at the drum | Line and the 3D-printed plastic gear of the winding mechanism destroyed | Pull the line out with soft brake only (README, 2026-09-30); camera + overwrap protection (WINCH-19) |

**Good practice (lesson learned):** the gears of the winding mechanism were first 3D-printed in plastic to check function and fit. They have since been replaced by **laser-sintered steel gears** for permanent use. Etienne's verdict: a good decision. Recommended approach for custom parts: prototype in printed plastic, then switch to a durable material (e.g. laser-sintered steel) for the part in use. Date of the replacement: *to be completed*.

## Test plan
- [ ] Etienne reviews each chapter.
- [ ] Checklists are used in the field once and refined.

## Open questions
- [x] Is a printable one-page checklist (PDF) wanted as well? → Yes, laminated on the winch: [WINCH-24](WINCH-24-printable-checklist.md).
- [ ] Is there DHV-relevant documentation to reference?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-09-28: created
- 2026-09-30: incident log started (rewind incident, line wrap after pulling out in neutral); lesson learned: printed plastic gears for prototyping, laser-sintered steel gears for use (Etienne). Several pilots / admin remote added to the scope.
- 2026-10-02: incident added: remote switched off too early after release, line fell onto the towing track (Etienne). README release section has a new warning.

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
9. **Incident log:** what went wrong and what was learned (e.g. the rewind incident).

## Out of scope
- Build instructions (the repo is explicitly not a build guide).

## Changes
- **Docs:** new `doc/manual.md`, README link, and slimming down the README usage section to a pointer.

## Test plan
- [ ] Etienne reviews each chapter.
- [ ] Checklists are used in the field once and refined.

## Open questions
- [ ] Is a printable one-page checklist (PDF) wanted as well?
- [ ] Is there DHV-relevant documentation to reference?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-09-28: created

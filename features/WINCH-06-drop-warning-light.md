# WINCH-06: Drop warning light

**Status:** Planned
**Type:** HW / Doc
**Priority:** P1
**Safety-relevant:** No
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
The relay on receiver IO12 was meant to switch the VESC cooling fan **and** a warning light (DHV regulations). The warning light won't be implemented; the relay becomes fan-only (automated in WINCH-07).

## Scope
- Hardware: remove / don't install the warning light on the relay output.
- Code comments and OLED text: "Fan/Light" → "Fan" (done together with WINCH-07, which rewrites that code anyway).
- README.md: remove warning light mentions from the relay/fan section and pin description.

## Out of scope
- Fan switching logic → WINCH-07.

## Changes
- **Hardware:** relay output → fan only.
- **Docs:** README.md.

## Test plan
- [ ] Fan still switches via the relay (verified in WINCH-07).

## Open questions
- [ ] Is anything else connected to the relay today?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| No warning light | Experience: not needed, adds complexity | 2026-09-28 |

## Log
- 2026-09-28: created

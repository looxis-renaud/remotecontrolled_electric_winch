# WINCH-24: Printable checklist (packing, setup, before launch)

**Status:** In Progress
**Type:** Doc
**Priority:** P1
**Safety-relevant:** Yes (a forgotten step, e.g. potentiometer not at the "off" end, line pulled out in neutral, or rewinding with too much pull, has already caused damage)
**Depends on:** –
**Created:** 2026-09-30 · **Last updated:** 2026-10-19

## Motivation
Etienne wants a short checklist to print, laminate and stick onto the winch: what to pack before leaving, how to set up the winch, what to check before pulling out the line and before launch. Several incidents came from single forgotten or wrong steps (line pulled out in neutral → line wrap; rewind with too much pull → azimuth system destroyed). A checklist on the winch makes the order repeatable, also for other pilots or helpers.

## Scope
- Markdown only, no PDF (Etienne's decision, 2026-10-19): a sorted, overviewable list is enough for now; `doc/checklist.md` is the source and the deliverable.
- Six lists, in German (on-site use; repo docs stay English, this file is the documented exception):
  1. **Day before**: charge batteries
  2. **Same day, before leaving**: what to pack + what to check (weather/airspace, per-pilot remote IDs, helper briefing)
  3. **Setup + preparing to pull out the line**
  4. **While pulling out the line and at the take-off spot, before launch**
  5. **During the (step) tow and at release**
  6. **After the tow / reeling in the line** (⚠ rewind rule: max. state 2/prePull)
- Link from README usage section and WINCH-09 (manual).

## Content

Current content lives in [doc/checklist.md](../doc/checklist.md) (6 lists, German), not duplicated here. Edit it directly; this feature file tracks status, decisions and open points.

## Out of scope
- Detailed explanations (README, WINCH-09 manual, vesc/vesc-tool-guide.md).

## Changes
- **Docs:** `doc/checklist.md` (created 2026-10-19), links from README and WINCH-09 still open.

## Test plan
- [ ] Printed and laminated, used on site once; missing or wrong items corrected.

## Open questions
- [ ] Order of setup steps as Etienne actually does them (e.g. camera before or after the remote)?
- [x] Format: PDF or Markdown? → Markdown only, no PDF (Etienne, 2026-10-19).
- [x] Language on the printout: German or English? → German (Etienne, 2026-10-19).
- [x] Is the 11 V monitor battery the same as the "LiPo for receiver/monitor", and does the winch camera need its own battery? → Same battery; the winch camera has no battery, it is powered by the VESC (Etienne, 2026-10-06).

## Log
- 2026-09-30: created (Etienne). Draft content from Etienne's list; candidates from today's findings added.
- 2026-10-06: new phase "0. Day before (ideally)" with the charging items from Etienne (11 V monitor battery, vario, 60 V winch battery, screwdriver battery, remote(s)). The old "All batteries charged" line in Packing was folded into it.
- 2026-10-19: Etienne restructured into 6 lists (day before; same day before leaving; setup + prep to pull out line; pulling out line + before launch; during tow + release; after tow / reeling in). Decided: Markdown only (no PDF), German. Added a dedicated "after tow / reeling in" list (⚠ rewind rule, max. state 2) that was missing from Etienne's restructuring request. Created `doc/checklist.md`.

# WINCH-14: Pull-value calibration procedure

**Status:** Idea
**Type:** Doc
**Priority:** P2
**Safety-relevant:** Yes (actual pull on the pilot)
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
From the old README ToDo and Robert's notes: the "kg" values in the code only match reality if the VESC current settings are calibrated. Robert measured about 3.6 A/kg with a QS260 hub motor using a suitcase scale. A documented, repeatable procedure is missing.

## Idea
- Document a measurement procedure (fixed anchor, luggage/crane scale, measure each state).
- Record measured kg per state in a table in the manual (WINCH-09).
- **No** VESC config change without Etienne's explicit decision (VESC is frozen); adjustments would go through `myMaxPull` / scale values, which is safety-relevant.

## Log
- 2026-09-28: created as idea (from old README ToDo)
- 2026-09-28: ToDo removed from README.md; this file is now the only place it is tracked.

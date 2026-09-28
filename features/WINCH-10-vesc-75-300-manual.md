# WINCH-10: Add Trampa VESC 75/300 user manual to `vesc/`

**Status:** Done
**Type:** Doc
**Priority:** P2
**Safety-relevant:** No
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
The controller in use is the Trampa VESC 75/300. Its manual (connectors, COMM/PPM pinout, limits) is needed for wiring, maintenance and troubleshooting and should be available offline next to the QS motor manuals already in `doc/`.

## Scope
- Find the official manual / datasheet from Trampa for the VESC 75/300 (R3).
- Check whether redistribution in a public repo is allowed. If yes, store it as a PDF in `doc/`; otherwise add the link (and a local copy outside the repo).
- Reference it from `vesc/readme.md` and `doc/parts-list.md`.

## Changes
- **Docs:** `doc/`, `vesc/readme.md`, `doc/parts-list.md`.

## Open questions
- [x] Does Etienne already have a copy? Yes, added as `vesc/VESC-75-300-MKIV-MANUAL.pdf`
- [ ] The manual is titled "MKIV", while the firmware binary targets hardware `75_300_R3`. Check which revision is printed on the controller and whether the manual matches (pinouts may differ between revisions).

## Log
- 2026-09-28: created
- 2026-09-28: manufacturer specs (product page) added to vesc/readme.md; the PDF manual itself is still open
- 2026-09-28: manual PDF added by Etienne as vesc/VESC-75-300-MKIV-MANUAL.pdf and linked from vesc/readme.md

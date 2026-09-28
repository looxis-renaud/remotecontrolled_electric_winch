# WINCH-10: Add Trampa VESC 75/300 user manual to `doc/`

**Status:** Planned
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
- [ ] Does Etienne already have a copy?

## Log
- 2026-09-28: created

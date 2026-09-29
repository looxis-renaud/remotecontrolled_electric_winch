# WINCH-10: Add Trampa VESC 75/300 user manual to `vesc/`

**Status:** Done
**Type:** Doc
**Priority:** P2
**Safety-relevant:** No
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-09-29

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
- [x] Does the manual match the installed controller? No: it describes the newer **MKVI** revision. The winch's controller (`75_300_R3`) has Mini-USB instead of USB-C, and its power-switch connector sits elsewhere. Noted in vesc/readme.md.
- [ ] The PDF filename says "MKIV", while the manual's title reads "MKVI". Rename the PDF if the filename is wrong.
- [ ] Are the motor's hall sensors connected to the VESC sensor port? (Etienne to check; TODO in vesc/readme.md.)

## Log
- 2026-09-28: created
- 2026-09-28: manufacturer specs (product page) added to vesc/readme.md; the PDF manual itself is still open
- 2026-09-28: manual PDF added by Etienne as vesc/VESC-75-300-MKIV-MANUAL.pdf and linked from vesc/readme.md
- 2026-09-29: wiring & safety notes from the manual (warning, power path, connectors, LED codes, power switch) summarised in vesc/readme.md; no separate Markdown transcript stored (PDF stays the source)
- 2026-09-29: Etienne: the manual covers the newer MKVI revision (installed controller has Mini-USB, power-switch connector elsewhere). Power switch on the winch: open = off, closed = on. vesc/readme.md adjusted

# WINCH-24: Printable checklist (packing, setup, before launch)

**Status:** Planned
**Type:** Doc
**Priority:** P1
**Safety-relevant:** Yes (a forgotten step, e.g. potentiometer not at the "off" end or line pulled out in neutral, has already caused damage)
**Depends on:** –
**Created:** 2026-09-30 · **Last updated:** 2026-10-06

## Motivation
Etienne wants a short checklist to print, laminate and stick onto the winch: what to pack before leaving, how to set up the winch, what to check before pulling out the line and before launch. Several incidents came from single forgotten or wrong steps (line pulled out in neutral → line wrap; rewind with too much pull → azimuth system destroyed). A checklist on the winch makes the order repeatable, also for other pilots or helpers.

## Scope
- One printable page per phase, or one page front/back, readable outdoors: large font, checkboxes, no explanations (details stay in the README / manual).
- Source as Markdown in `doc/` (e.g. `doc/checklist.md`) plus a print-ready PDF.
- Phases:
  0. **Day before** (ideally): charge batteries
  1. **Packing** (before leaving home)
  2. **Setup** (on site)
  3. **Before pulling out the line**
  4. **Before launch**
  5. **After the tow / packing up** (optional, to be decided)
- Link from README usage section and WINCH-09 (manual).

## Draft content (from Etienne, 2026-09-30; to be completed and ordered)

**0. Day before (ideally)**
- [ ] 11 V LiPo for the camera receiver + monitor (cockpit) charged
- [ ] Vario charged
- [ ] 60 V winch battery charged
- [ ] Cordless screwdriver battery charged
- [ ] Winch remote(s) charged

**1. Packing**
- [ ] Winch
- [ ] Line parachute (drogue)
- [ ] Winch battery
- [ ] Controller (VESC) / receiver unit
- [ ] Remote(s); admin remote (red) if towing with several pilots
- [ ] Ratchet straps, ground screws and cordless screwdriver (to fix the winch to the ground)
- [ ] Antenna (receiver)
- [ ] Camera (winch, powered by the VESC), camera receiver + monitor (cockpit), 11 V LiPo for receiver/monitor
- [ ] Paraglider, harness with reserve, cockpit

**2. Setup**
- [ ] Fix the winch to the ground with ratchet straps (and screws)
- [ ] Set up and connect the antenna
- [ ] Switch on controller (VESC) and receiver
- [ ] VESC LED not red (otherwise read faults, see vesc/vesc-tool-guide.md)
- [ ] **Potentiometer fully left (off end)**
- [ ] Switch on the remote, link OK (RSSI on the OLED), winch mode **soft brake "B -1" (−7 kg)**
- [ ] Camera and monitor on, image of the drum visible

**3. Before pulling out the line**
- [ ] **Soft brake active (B -1), never neutral** (neutral → drum overruns → line wrap)
- [ ] Pull out the line with the drogue, walk steadily

**4. Before launch**
- [ ] Line lies properly on the drum (monitor)
- [ ] Line, weak link, connection to the harness checked
- [ ] Remote in hand / fixed, OLED shows the link
- [ ] Potentiometer still at the off end

**5. After the tow** (to be decided)
- [ ] Rewind with **max. state 2 (prePull)**
- [ ] Remote stays **on** until the line is fully in and AutoStop has stopped the drum
- [ ] ...

## Candidates to add (to be decided with Etienne)
- Sensor cable (hall + motor temperature) plugged in: without it the VESC reports over-temperature and gives no current.
- Remote OLED plausible: battery %, motor temperature.
- Wind / weather / airspace, helper briefed (if any).
- Several pilots: each pilot's own remote (own ID and max pull), admin remote rules (README "Several pilots / several remotes").

## Out of scope
- Detailed explanations (README, WINCH-09 manual, vesc/vesc-tool-guide.md).

## Changes
- **Docs:** `doc/checklist.md` + PDF, links from README and WINCH-09.

## Test plan
- [ ] Printed and laminated, used on site once; missing or wrong items corrected.

## Open questions
- [ ] Format: A5 or A4? One sheet front/back or one sheet per phase?
- [ ] Language on the printout: German or English? (Repo docs are English; a German printout for use on site is possible.)
- [ ] Order of setup steps as Etienne actually does them (e.g. camera before or after the remote)?
- [x] Is the 11 V monitor battery the same as the "LiPo for receiver/monitor", and does the winch camera need its own battery? → Same battery; the winch camera has no battery, it is powered by the VESC (Etienne, 2026-10-06).

## Log
- 2026-09-30: created (Etienne). Draft content from Etienne's list; candidates from today's findings added.
- 2026-10-06: new phase "0. Day before (ideally)" with the charging items from Etienne (11 V monitor battery, vario, 60 V winch battery, screwdriver battery, remote(s)). The old "All batteries charged" line in Packing was folded into it.

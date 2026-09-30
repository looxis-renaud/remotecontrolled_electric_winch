# WINCH-21: Add CAD files of the winding gears (Fusion + STL)

**Status:** Planned
**Type:** HW / Doc
**Priority:** P2
**Safety-relevant:** No
**Depends on:** –
**Created:** 2026-09-30 · **Last updated:** 2026-09-30

## Motivation
The gears of the winding mechanism were designed in Fusion, first 3D-printed in plastic to check function and fit, and are now laser-sintered steel for permanent use. The design files exist only on Etienne's computer. With them in the repo, a replacement gear can be ordered again at any time.

## Scope
- Add the original Fusion file (`.f3d`) and the STL file(s) of the gears to `doc/` (e.g. `doc/winding-gears/`).
- Short README there: what the part is, material (laser-sintered steel; plastic only for fit tests), where it was ordered, notes for re-ordering.
- Link from `doc/parts-list.md` (section "Winding Mechanism").

## Out of scope
- Design changes to the gears.

## Changes
- **Docs:** new files in `doc/`, parts-list link.

## Test plan
- [ ] STL opens in a viewer / slicer; Fusion file opens in Fusion.

## Open questions
- [ ] Supplier and order details of the laser-sintered steel gears?
- [ ] Date of the replacement (plastic → steel)?
- [ ] Other CAD files worth adding (e.g. further parts of the winding mechanism or azimuth system)?

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Prototype in printed plastic, then laser-sintered steel for use | Plastic is cheap for checking function and fit; a plastic gear was destroyed in a line wrap, steel lasts | 2026-09-30 |

## Log
- 2026-09-30: created (ToDo for Etienne: add Fusion + STL files).

# WINCH-19: Winch camera & line overwrap protection

**Status:** Planned
**Type:** HW
**Priority:** P1
**Safety-relevant:** Yes (a line wrap at the drum destroys the line and the winding gear and can end a tow abruptly)
**Depends on:** –
**Created:** 2026-09-30 · **Last updated:** 2026-09-30

## Motivation
When towing alone, the pilot is far away from the winch at launch and cannot see whether the line lies properly on the drum. Once the line was pulled out in neutral (state 0): when Etienne stopped after walking faster, the drum overran and unwound loose turns right at the drum. On launch this caused a line wrap that destroyed the line and the 3D-printed gear of the winding mechanism. (The gears have since been replaced by laser-sintered steel gears.)

The usage rule "pull the line out with soft brake only" is now in the README (2026-09-30). This feature adds two hardware measures on top of it.

## Scope
1. **Camera at the winch**
   - View of the drum and the line guide (barrel-cam winding / azimuth system).
   - Live image on a portable monitor carried to the launch site: check before launch that the line lies properly on the drum.
   - Record the tow at the winch to document it (and analyse incidents).
2. **Overwrap protection**
   - Mechanical guard that keeps loose line from jumping over the drum flange or wrapping around the axle/gear, e.g. a close-fitting guard or line holder over the drum.
   - Must not hinder normal winding and must not add friction to the line.

## Out of scope
- Changes to the winch electronics or firmware (camera is a separate, independent system).
- Automatic detection of a bad line lay (possible later idea).

## Changes
- **Hardware:** camera + mount + power, portable monitor; overwrap guard (design, build, mount).
- **Docs:** parts list, photos in `doc/`, pre-flight check in the manual (WINCH-09): look at the drum on the monitor before launch.

## Test plan
### Bench (workshop, no pilot)
- [ ] Camera: image of drum and line guide visible on the monitor at the expected distance to launch (range test).
- [ ] Camera power does not drain or disturb the winch electronics (separate supply or clean supply).
- [ ] Overwrap guard: wind in and pull out the full line length several times, no rubbing or jamming.

### Field (real towing)
- [ ] Camera: before launch the line lay can be judged on the monitor; tow recorded.
- [ ] Overwrap guard: several tows without problems.

## Open questions
- [ ] Camera type: FPV camera + analog/digital video link (low latency, separate monitor), action cam with WiFi to a phone, or IP camera? Range needed (winch → launch site)?
- [ ] Recording: on the camera itself or on the monitor/receiver side?
- [ ] Power: own battery or from the winch battery via a DC-DC converter (then fuse and separate from the VESC aux budget)?
- [ ] Mount: where on the frame, protected from the line and weather?
- [ ] Overwrap guard: design (material, distance to the drum, how to handle the barrel-cam line guide)?
- [x] Replace the 3D-printed gear with a stronger part? → Done: the gears of the winding mechanism are now laser-sintered steel (Etienne, 2026-09-30).

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-09-30: created (Etienne). Background: line wrap after pulling the line out in neutral, which destroyed the line and the 3D-printed gear.

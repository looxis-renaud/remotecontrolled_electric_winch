# WINCH-19: Winch camera & line overwrap protection

**Status:** In Progress
**Type:** HW
**Priority:** P1
**Safety-relevant:** Yes (a line wrap at the drum destroys the line and the winding gear and can end a tow abruptly; the camera is powered from the VESC's 5 V output)
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
- Firmware changes (the camera system is independent of transmitter/receiver).
- Automatic detection of a bad line lay (possible later idea).

## Camera system (Etienne, 2026-09-30: ordered, received, tested)

| Part | Details | Source |
|------|---------|--------|
| Camera + video transmitter | BetaFPV P1 Air Unit HD, 5 V | rotorama |
| Video receiver (ground unit) | HGLRC Draco VRX Ground Unit, DC input 7–36 V (runs on a 2S–6S battery), records to SD card | rotorama |
| Monitor | Elecrow 5 inch HDMI monitor, 5 V via USB-C | Amazon |
| Battery (mobile) | LiPo 11.1 V (3S), 5200 mAh, supplies receiver and monitor | *to be completed* |
| DC-DC converter | LiPo voltage → 5 V for the monitor | *to be completed* |

- **Winch side:** the camera is mounted on the winch, in a 3D-printed housing (printed, tested, mounted).
- **Pilot side:** receiver + monitor in a 3D-printed housing (printed and tested), to be mounted on the harness cockpit with hook-and-loop fastener and a safety cord; the battery goes into the cockpit pocket. The receiver runs directly on the 3S LiPo, the monitor via the DC-DC converter.
- Recording: on the receiver's SD card (pilot side).

## Changes
- **Hardware:** camera + housing on the winch, powered from the VESC 5 V output; receiver/monitor unit for the cockpit; overwrap guard (design, build, mount).
- **Docs:** parts list ("Camera / video link"), photos in `doc/`, pre-flight check in the manual (WINCH-09): look at the drum on the monitor before launch.

## Remaining ToDos
- [ ] Connect the camera to the 5 V supply of the VESC (see the power check below first).
- [ ] Mount the receiver/monitor unit on the harness cockpit with hook-and-loop fastener **and safety cord**; battery into the cockpit pocket. (The earlier cockpit monitor was lost in flight, see WINCH-04, so the safety cord matters.)
- [ ] Overwrap guard: design, build, mount.

## Test plan
### Bench (workshop, no pilot)
- [x] Camera, receiver and monitor work together (Etienne, 2026-09-30).
- [ ] **Power check before connecting the camera to the VESC:** current draw of the camera at 5 V (measure, or from the data sheet). The VESC aux outputs (3.3 V, 5 V, 12 V) share **1 A in total** (see `vesc/readme.md`), together with everything else powered from them (e.g. potentiometer, possibly receiver or fan relay, depending on the build). If the camera plus the rest comes close to 1 A, power it from the battery via a separate DC-DC converter with fuse instead.
- [ ] Camera connected to the VESC 5 V: winch behaves normally in all states (remote, PPM, UART telemetry, potentiometer / manual mode not triggered).
- [ ] Image of drum and line guide visible on the monitor at the distance from winch to launch (range test).
- [ ] LoRa link (868 MHz) unaffected with the video link running (RSSI on both OLEDs as before).
- [ ] Recording to the SD card works.
- [ ] Runtime of receiver + monitor on the 5200 mAh LiPo: ___ h.
- [ ] Overwrap guard: wind in and pull out the full line length several times, no rubbing or jamming.

### Field (real towing)
- [ ] Camera: before launch the line lay can be judged on the monitor; tow recorded.
- [ ] Monitor on the cockpit doesn't distract or obstruct during launch and tow; stays attached.
- [ ] Overwrap guard: several tows without problems.

## Open questions
- [x] Camera type? → Digital FPV system: BetaFPV P1 Air Unit HD + HGLRC Draco VRX ground unit + Elecrow 5" HDMI monitor (Etienne, 2026-09-30).
- [x] Recording? → On the receiver's SD card.
- [x] Power? → Camera from the VESC 5 V output; receiver + monitor from a 3S LiPo 5200 mAh (monitor via DC-DC to 5 V).
- [x] Mount? → Camera in a printed housing on the winch (mounted).
- [ ] Current draw of the camera at 5 V vs. the VESC's 1 A aux budget?
- [ ] Video transmit power / channel: within the legal limits for 5.8 GHz in Germany (usually 25 mW EIRP)?
- [ ] Order links for LiPo and DC-DC converter (parts list)?
- [ ] Overwrap guard: design (material, distance to the drum, how to handle the barrel-cam line guide)?
- [x] Replace the 3D-printed gear with a stronger part? → Done: the gears of the winding mechanism are now laser-sintered steel (Etienne, 2026-09-30).

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Digital FPV camera system with its own receiver and monitor | Low latency live image at the launch site, recording on SD card, independent of the winch radio link | 2026-09-30 |

## Log
- 2026-09-30: created (Etienne). Background: line wrap after pulling the line out in neutral, which destroyed the line and the 3D-printed gear.
- 2026-09-30 (Etienne): camera system ordered, received and tested (parts see above). Housings printed, tested and mounted; camera mounted on the winch. Remaining: camera power from the VESC 5 V, receiver/monitor on the harness cockpit. Status → In Progress.

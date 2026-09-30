# WINCH-23: VESC Tool user guide

**Status:** In Progress
**Type:** Doc
**Priority:** P1
**Safety-relevant:** No (documentation; describes safety-relevant settings but changes nothing)
**Depends on:** WINCH-17
**Created:** 2026-09-30 · **Last updated:** 2026-09-30

## Motivation
Etienne wants to understand how to work with VESC Tool (desktop and smartphone) and be able to look everything up again months later: reading faults, realtime data, reading/writing settings, backups, the temperature sensor setting, hall detection, ERPM and the speed ranges of hall / sensorless / HFI. Based on the VESC Tool session of 2026-09-30.

## Scope
- [x] `vesc/vesc-tool-guide.md` (desktop VESC Tool 3.01, FW 5.3), referencing the current configs `vesc/260930_*.xml`.
- [ ] Smartphone app section (version, connection, available functions).
- [ ] Etienne reviews the guide against VESC Tool (menu names and button positions in 3.01).

## Out of scope
- Changing VESC settings.

## Changes
- **Docs:** new `vesc/vesc-tool-guide.md`; links from `vesc/readme.md`, README.md, CLAUDE.md. Tachometer / line-length hypothesis added to WINCH-08 and CLAUDE.md.

## Open questions
- [ ] Smartphone app: version, FW 5.3 compatibility, connection (Bluetooth module? USB-OTG?).
- [ ] Is the "Rotor Position" check in section 5 useful in practice?

## Log
- 2026-09-30: created. Guide written from today's session: connecting, read/write, backups, terminal and fault codes, realtime data, temperature sensor type, ERPM table and hall / sensorless / HFI ranges, tachometer, hall detection, don'ts, worked example. While calculating the ERPM table: hypothesis that the ~0.7× line length and the autostop distance come from the drum size (see WINCH-08).

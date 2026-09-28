# Feature Index

> Central roadmap for all hardware, firmware and documentation changes on the winch.
> One file per feature: `WINCH-NN-<slug>.md` (template: [TEMPLATE.md](TEMPLATE.md)).

## Status Legend
- **Idea**: noted, not yet specified
- **Planned**: scope, changes and test plan written
- **In Progress**: being built / coded / written
- **Bench Test**: built, being tested in the workshop (winch on the stand, no pilot)
- **Field Test**: being tested in real towing operation
- **Done**: tested and in use
- **Dropped**: decided against; code/docs archived where applicable

**Safety-relevant** = touches pull/brake behaviour, failsafe, autostop, the LoRa protocol or anything a pilot relies on. These always need Etienne's explicit confirmation and a bench test before a field test.

## Suggested order
**WINCH-01** → 02 → 03 → 04 → 08 → 05 + 06 + 07 (one flash, struct change) → 09 → rest

## Features

| ID | Feature | Type | Status | Priority | Safety | Depends | Spec |
|----|---------|------|--------|----------|--------|---------|------|
| WINCH-01 | ⚠ Fix dangerous README rewind instruction (max. state 2) | Doc | In Progress | P0 | Yes | – | [WINCH-01](WINCH-01-fix-readme-rewind-instruction.md) |
| WINCH-02 | Sync repo with latest local code (other machine) | FW | Planned | P0 | Yes | – | [WINCH-02](WINCH-02-sync-latest-local-code.md) |
| WINCH-03 | Source-file cleanup (one `.ino` per sketch) & drop PlatformIO | FW/Doc | Planned | P0 | No | WINCH-02 | [WINCH-03](WINCH-03-source-cleanup-drop-platformio.md) |
| WINCH-04 | Archive cockpit monitor → `old/` | Doc/FW | Planned | P0 | No | WINCH-03 | [WINCH-04](WINCH-04-archive-cockpit-monitor.md) |
| WINCH-05 | Drop emergency line cutter | FW/Doc | Planned | P1 | Yes | WINCH-02, WINCH-03 | [WINCH-05](WINCH-05-drop-emergency-line-cutter.md) |
| WINCH-06 | Drop warning light (relay = fan only) | HW/Doc | Planned | P1 | No | – | [WINCH-06](WINCH-06-drop-warning-light.md) |
| WINCH-07 | Automatic cooling fan by operating mode | FW | Planned | P1 | Yes | WINCH-05, WINCH-06 | [WINCH-07](WINCH-07-automatic-cooling-fan.md) |
| WINCH-08 | Autostop documentation & verification (poti, ADC, FW version) | Doc/VESC | Planned | P0 | Yes | WINCH-01 | [WINCH-08](WINCH-08-autostop-docs-verification.md) |
| WINCH-09 | User & maintenance manual | Doc | Planned | P1 | Yes | WINCH-01, WINCH-08 | [WINCH-09](WINCH-09-user-maintenance-manual.md) |
| WINCH-10 | Add Trampa VESC 75/300 user manual to `doc/` | Doc | Planned | P2 | No | – | [WINCH-10](WINCH-10-vesc-75-300-manual.md) |
| WINCH-11 | New enclosure & cable management | HW | Idea | P1 | No | – | [WINCH-11](WINCH-11-enclosure-cable-management.md) |
| WINCH-12 | Remote case redesign + LiPo pouch cells | HW | Idea | P2 | No | – | [WINCH-12](WINCH-12-remote-case-lipo.md) |
| WINCH-13 | TX/RX pairing / encryption | FW | Idea | P3 | Yes | WINCH-02 | [WINCH-13](WINCH-13-tx-rx-pairing-encryption.md) |
| WINCH-14 | Pull-value calibration procedure | Doc | Idea | P2 | Yes | – | [WINCH-14](WINCH-14-pull-value-calibration.md) |
| WINCH-15 | Pin / board review (tech-debt list) | FW | Idea | P2 | Yes | WINCH-02 | [WINCH-15](WINCH-15-pin-board-review.md) |

<!-- Add features above this line -->

## Next Available ID: WINCH-16

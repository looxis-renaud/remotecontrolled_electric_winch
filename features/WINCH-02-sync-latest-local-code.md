# WINCH-02: Sync repo with latest local code

**Status:** Done
**Type:** FW
**Priority:** P0
**Safety-relevant:** Yes (the flashed code is what pilots rely on)
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
The repo code is about two years old (last sketch changes: `receiver.ino` 2024-04-04, `transmitter.ino` 2024-02-16, `transmitter_with-monitor-support.cpp` 2024-04-16). Etienne may have newer versions with small changes on another machine. Those may be exactly what is flashed on the winch today. All later code features must build on the code that is actually in use.

## Scope
- Collect the transmitter and receiver sketches from Etienne's other machine.
- Diff them against the repo versions (`transmitter/transmitter.ino`, `transmitter/transmitter_with-monitor-support.cpp`, `receiver/receiver.ino`, `receiver/LiPoCheck.*`).
- Determine which variant is actually flashed on the handheld and on the receiver (the OLED start screen and behaviour can help, e.g. does the transmitter try ESP-NOW?).
- Commit the newest/in-use versions as the new baseline, with a commit message listing the differences.

## Out of scope
- Cleaning up file names / PlatformIO → WINCH-03.
- Functional changes.

## Changes
- **Firmware transmitter / receiver:** replace with the in-use versions (no logic changes by Claude).

## Test plan
- [x] Diff reviewed together; every difference explained.
- [ ] Baseline compiles in the Arduino IDE (Etienne). → Deferred by Etienne (2026-09-28); to be done in WINCH-03 when the sketch folder is cleaned up.
- [ ] OLED shows `3-B: …` on the handheld (confirms ID 3 / maxPull 95 is flashed). → Deferred by Etienne (2026-09-28).

## Open questions
- [x] Where are the newer files (machine, path)? → Etienne's other machine; pasted into the session 2026-09-28.
- [x] Which transmitter variant is flashed: with or without monitor support? → Most likely with monitor support (ID 3, maxPull 95), per Etienne. Verify via OLED (`3-B: …`).
- [x] Is a VESC config backup from the actual controller available (to compare with `vesc/240601_*.xml`)? → Yes, read out on 2026-09-30: practically identical to the June 2024 backup (then changed in WINCH-17, now `vesc/260930_*.xml`; the 2024 backups are in `old/vesc-configs/`).

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Etienne's local transmitter (2025-08-24) becomes the baseline, stored as `transmitter/transmitter_with-monitor-support.cpp` | Closest repo file; most likely the flashed version. File naming is left to WINCH-03. | 2026-09-28 |
| Receiver unchanged | Local files identical to repo | 2026-09-28 |
| `receiver-pincorrected.txt` ignored | No functional difference; not relevant for now (Etienne) | 2026-09-28 |

## Findings
### Transmitter (Etienne's local `transmitter.ino`, file date 2025-08-24)
- Is the **monitor-support variant** (ESP-NOW), not the plain `transmitter.ino` from the repo.
- Code is identical to commit `8433fe0` (2024-03-21, `transmitter_with-monitor-support.cpp`) except:
  - `myID = 3` (repo: 8)
  - ⚠ `myMaxPull = 95` (repo and CLAUDE.md: 85). Resulting targets: state 2 = 17 kg, 3 = 52, 4 = 76, 5 = 95 (with 85: 15 / 46 / 68 / 85).
  - comment edits only
- It does **not** contain the later `loraConnect` changes from `a442935`/`a57a14f` (2024-04-16), which are in the repo's current `.cpp`. The ESP-NOW struct therefore differs from the repo version (no `loraConnect` field), so a monitor flashed from the repo would not match.
- ESP-NOW peer MAC `DC:DA:0C:5A:59:58` (T-Display monitor); repo `.cpp` has `DC:DA:0C:58:FE:B8`.
- LoRa structs (`LoraTxMessage`, `LoraRxMessage`) are identical to the repo, so no protocol change vs. the repo receiver.
- Etienne (2026-09-28): most likely the flashed version. `myMaxPull = 95` was chosen deliberately because it matches his take-off weight (pull ≈ pilot take-off weight). Not yet verified on the device. Quick check: the OLED's top line starts with the transmitter ID (`3-B: …` vs `8-B: …`), and the pull line shows `target/current kg` (e.g. state 2 → `17/…` with maxPull 95, `15/…` with 85).
- Existing issues in this code (same in repo, not changed, for later features):
  - `OnDataRecv` copies `setMaxPull` from the monitor into `myMaxPull` without any range check. `targetPull` is `int8_t`, so values above 127 would overflow. Relevant for WINCH-04 (monitor retirement).
  - If `esp_now_init()` or `esp_now_add_peer()` fails, `setup()` returns early, **before** the OLED and the button handlers are set up. The transmitter would then keep sending soft brake but could not be operated.

### Receiver (Etienne's local files, file date 2024-04-04)
- `receiver.ino`, `LiPoCheck.cpp`, `LiPoCheck.h`: **identical to the repo** (only whitespace / trailing blank lines differ). The repo receiver is already the local 2024-04-04 state.
- `receiver-pincorrected.txt` (second local file): despite the name, **no pin changes**. Compared by reading, not by diff: pins, structs, values, failsafe, autostop, smoothing and PPM output are the same as `receiver.ino`. The only difference is that four serial debug prints in the display block are active (`Relay On`, `Relay OFFFF`, `Line Cutter Ready`, `EMERGENCY`). Open: was a pin fix (e.g. the `RST 14` / `VESC_RX 14` collision) planned but never saved? Which of the two is flashed on the receiver?

## Log
- 2026-09-28: created
- 2026-09-28: local transmitter (2025-08-24) diffed against repo, see Findings.
- 2026-09-28: local receiver files diffed: identical to repo.
- 2026-09-28: `transmitter/transmitter_with-monitor-support.cpp` replaced with Etienne's local version. Not compiled yet.
- 2026-09-28: README.md and CLAUDE.md updated (current transmitter file, pull values for `myMaxPull = 95`). Committed. Remaining: compile in Arduino IDE, verify ID 3 on the handheld OLED.
- 2026-09-28: set to Done by Etienne. Compile and on-device check deferred (see Test plan); baseline is not compile-verified.

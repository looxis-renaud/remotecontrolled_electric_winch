# WINCH-01: ⚠ Fix dangerous README rewind instruction

**Status:** Done
**Type:** Doc
**Priority:** P0
**Safety-relevant:** Yes
**Depends on:** –
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
The README usage section ("C) Release") currently says:

> Go To defaultPull (7kg pull value) before you release. Release and move back to fullPull to rewind the line.

This is **wrong and dangerous**. It was Etienne's own earlier assumption. Rewinding with full pull once led to the autostop brake not being able to stop the drum in time: the carabiner was pulled into the azimuth system, the line broke and tangled, and the whole winch had to be rebuilt. (It may have been a cascade of errors, but rewinding with high pull was a key factor.)

## Scope
- Replace the release/rewind instruction in README.md: after release, rewind with **maximum state 2 (prePull)**, never higher.
- Add a prominent warning block explaining why (brake can't hold → carabiner pulled into the azimuth system / line snaps; has happened once).
- Adjust the neighbouring sentence about autostop so it doesn't imply that autostop protects against high-speed rewinds.

## Out of scope
- Poti wiring and positions, the full autostop description → WINCH-08.
- Rewriting the rest of the usage section → WINCH-09 (manual).
- Any code change (e.g. limiting the pull after release in firmware). This could be a later idea, but it is safety-relevant and needs its own feature.

## Changes
- **Docs:** README.md, section "usage → C) Release" only.

## Test plan
- [x] `git diff README.md` shows only the release section changed.
- [x] Etienne reads and approves the wording. (Set to Done by Etienne, 2026-09-28.)

## Open questions
- [ ] Should the transmitter firmware *enforce* this later (e.g. cap at state 2 once line length is short, or after a release is detected)? → new feature if yes.

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|
| Rewind with max. state 2 (prePull ≈ 15 kg) | Real incident: stronger pull overran the autostop brake and destroyed azimuth system / line | 2026-09-28 |
| Done first, before all other features | Safety-critical and the README is public; others may copy the wrong procedure | 2026-09-28 |

## Log
- 2026-09-28: created
- 2026-09-28: README "C) Release" rewritten + warning block added; awaiting Etienne's review of the wording
- 2026-09-28: prePull value in the rule updated to ~17 kg (WINCH-02: `myMaxPull = 95`). Rule itself unchanged: max. state 2.
- 2026-09-28: set to Done by Etienne. Open question on firmware enforcement stays open for a possible future feature.

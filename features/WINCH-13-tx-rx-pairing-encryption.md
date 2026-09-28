# WINCH-13: TX/RX pairing / encryption

**Status:** Idea
**Type:** FW
**Priority:** P3
**Safety-relevant:** Yes (protocol change)
**Depends on:** WINCH-02
**Created:** 2026-09-28 · **Last updated:** 2026-09-28

## Motivation
From the old README ToDo: some form of encryption or password so that only the paired transmitter can control the winch. Today any LoRa sender with a matching packet size and ID-lock timing could take over (admin ID 0 always can).

## Open questions
- [ ] Is this a real risk at the launch sites used, or over-engineering (see the "keep it simple" lesson)?
- [ ] Minimal option: a shared "network key" byte/checksum in the struct instead of real encryption?

## Log
- 2026-09-28: created as idea (from old README ToDo)
- 2026-09-28: ToDo removed from README.md; this file is now the only place it is tracked.

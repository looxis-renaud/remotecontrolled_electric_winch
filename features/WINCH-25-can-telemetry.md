# WINCH-25: Telemetry over the VESC CAN port

**Status:** Idea
**Type:** HW / FW / VESC
**Priority:** P3
**Safety-relevant:** Yes (brings the tachometer back to the receiver, which re-activates the receiver autostop taper; needs a VESC config change)
**Depends on:** WINCH-16, WINCH-08
**Created:** 2026-10-03 · **Last updated:** 2026-10-03

## Motivation
The VESC COMM UART no longer answers (WINCH-16): ESP32 pins and cables are proven fine by a loopback test, but the VESC sends nothing back. The VESC has an unused CAN port. Etienne asked whether the telemetry could go through it instead.

## How it would work (not built, not tested)
- CAN is a different bus, not UART: the UART link cannot simply be "rerouted" to the CAN port.
- The VESC can **broadcast status messages on CAN** by itself (App Settings → General → *Can Status Message Mode*, today `CAN_STATUS_DISABLED`, CAN baud 500K). These messages contain, among others, RPM, current and duty cycle (status 1), temperatures (status 4), tachometer and input voltage (status 5). The receiver would only listen, no requests needed.
- The ESP32 has a built-in CAN controller (TWAI, included in the esp32 core 2.0.15, no extra library). It needs an external **3.3 V CAN transceiver module** (e.g. SN65HVD230) on two free GPIOs, wired CANH/CANL/GND to the VESC CAN port, with 120 Ω termination at the ends of the bus.

## Scope (if pursued)
- Hardware: transceiver module, wiring, termination.
- VESC: enable CAN status messages (**VESC config change: only with Etienne's explicit decision**, config is frozen by default).
- Receiver: read status 1/4/5 frames instead of `getVescValues()`, same values into the LoRa ack.

## Out of scope
- Sending commands to the VESC over CAN (pull/brake stays on PPM).

## Open questions
- [ ] Does the receiver autostop taper (raw tachometer 2–40) make sense once the tachometer arrives again? Units vs. the patch (1500 ≈ 15 m) are unclear (WINCH-08). Decide before re-enabling.
- [ ] Ask Trampa whether the 75/300 R3 COMM UART can be repaired, or has a second UART, before building this.

## Decision log
| Decision | Rationale | Date |
|----------|-----------|------|

## Log
- 2026-10-03: created as an idea after WINCH-16 debugging was paused (Etienne's question about the CAN port).

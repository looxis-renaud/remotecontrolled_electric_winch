# QS Motor 12kW 260 V4 (hub motor)

The winch drum is driven directly by a QS Motor 12kW 260 V4 hub motor. The motor housing (outer rotor) turns with the drum; the axle is fixed. The motor is driven by the Trampa VESC 75/300 in FOC mode, see [vesc/readme.md](../../vesc/readme.md).

This page collects what is known about the motor. Values marked *to be completed* are still missing. Please fill them in when known and name the source.

## Files in this folder

| File | Content |
|------|---------|
| [QSMOTOR-Motor-Manual-V1.57.pdf](QSMOTOR-Motor-Manual-V1.57.pdf) | QS general motor manual: wire colours, pole pairs, hall and temperature sensor checks, temperature limits, waterproofing, mounting, hall replacement |
| [QSMOTOR-Connection-Manual-Kelly.pdf](QSMOTOR-Connection-Manual-Kelly.pdf) | QS connection manual for car hub motors with Kelly controllers: phase and hall wiring, spare hall set |
| [sensors-troubleshooting.md](sensors-troubleshooting.md) | Test procedure for the temperature sensor and the three hall sensors (German) |

## Identification

- **Model:** QS Motor 12kW 260 V4 hub motor, ordered as version "72V 100kph" ([AliExpress listing](https://www.aliexpress.com/item/4001237537247.html), see [parts list](../parts-list.md)).
- According to the QS manual, V3/V4 motors are marked "WP" on the motor.
- Serial number, purchase date, exact order specification: *to be completed*.

## Known data

| Item | Value | Source |
|------|-------|--------|
| Rated power | 12 kW | Product name |
| Pole pairs | **16** (32 magnets) | QS manual p. 3 (V3/V4 motors), VESC configs `si_motor_poles = 32` |
| Phase angle | 120° | QS manual p. 3 |
| Phase wires | yellow = U/A, green = V/B, blue = W/C | QS manual p. 1, Kelly manual |
| Hall sensors | 3 hall sensors, 5 V supply. **Two sensor sets** (each 3 halls + temperature sensor), one of them a spare | QS manual p. 1, Kelly manual |
| Sensor set 1 | Fitted with our own plug, in use. All 3 hall sensors measured intact | Etienne, 2026-09-30 |
| Sensor set 2 | Original QS plug (no matching counterpart), never connected, unchecked | Etienne, 2026-09-30 |
| Temperature sensor | **Most likely KTY83-122.** Measured 0.97 kΩ at ~25 °C (KTY83-122 nominal 1.0 kΩ; a PT1000 would be ~1.1 kΩ, a 10k NTC ~10 kΩ). Heating test (resistance must rise ~0.8 %/°C) *to be completed* | QS manual p. 3, Etienne's measurement 2026-09-30 |
| Waterproofing | IP65 (hub motors) | QS manual p. 5 |
| QS temperature limits (V2/V3/V4) | 130 °C inside the motor (for 30 s) → limit current to 50 %; 145 °C → shut down, resume at 110 °C | QS manual p. 11 |
| Phase resistance R | ≈ 4.3–5.8 mΩ | VESC motor detection: `240601_motor_config.xml` (4.34 mΩ), Robert's `vesc_motor_config_12kw_260_V4.xml` (5.8 mΩ) |
| Inductance L | ≈ 16–18 µH (Ld–Lq ≈ 4.7 µH) | same configs |
| Flux linkage λ | ≈ 25.6–26.1 mWb | same configs |
| Torque constant Kt | ≈ 0.63 Nm/A (derived: 1.5 × 16 pole pairs × λ). With the 0.43 m drum that is ≈ 3.4 A motor current per kg of line pull, consistent with the ~3.6 A/kg used for `myMaxPull` | Calculated estimate, not measured |
| KV (rpm/V) | *to be completed* | QS spec sheet or VESC Tool |
| Rated / max phase current, rated torque | *to be completed* | QS spec sheet |
| Rated voltage range, max rpm | *to be completed* (ordered as the 72 V version) | QS spec sheet |
| Weight | *to be completed* | |

## Sensor cable wiring (sensor set 1)

Confirmed by Etienne, 2026-09-30.

> ⚠ **The plug colours do not match the motor wire colours.** Red on the plug is a hall signal, green on the plug is +5 V. Always go by this table, not by colour conventions.

| Motor wire colour | Plug wire colour | Function |
|-------------------|------------------|----------|
| yellow | yellow | Hall A |
| green | white | Hall B |
| blue | red | Hall C |
| black | black | GND (halls and temperature sensor) |
| red | green | +5 V (hall supply) |
| silver (QS manual: "transparent") | blue | Temperature sensor (measured against GND) |

- Hall A/B/C follow the phase wire colours (yellow = A, green = B, blue = C). Connecting Hall A/B/C to H1/H2/H3 of the VESC is the natural choice. The assignment is not critical, because the VESC hall detection works out the order itself.
- Pin positions in the VESC sensor port and which plug colour goes to which VESC pin: *to be completed*. The Trampa manual in `vesc/` shows a newer controller revision, so check on the actual device.
- The QS manual warns that the hall sensors can be destroyed by static discharge above 5 V: don't touch the contacts without grounding yourself.

## VESC settings for this motor

The VESC configuration is documented in [vesc/readme.md](../../vesc/readme.md). That file is the authority. Motor-related values in the configs in `vesc/`:

| Setting | Value | Note |
|---------|-------|------|
| `motor_type` | 2 (FOC) | all configs |
| `l_current_max` / `l_current_min` | 250 A / −250 A | Robert's 260 V4 config and Etienne's 2024 config |
| `l_temp_motor_start` / `l_temp_motor_end` | 75 °C / 85 °C | Much more conservative than the QS limits above. Only works with the sensor cable connected |
| `foc_sensor_mode` | Robert 2022: 2 (Hall); Etienne 2024: 0 (sensorless) | See [WINCH-17](../../features/WINCH-17-hall-sensors-foc.md) |
| `m_motor_temp_sens_type` | Robert: 2; Etienne 2024: 0 | Must match the KTY83-122 once confirmed (WINCH-17) |
| `si_motor_poles` | 32 | = 16 pole pairs |
| `si_wheel_diameter` | 0.43 m (Etienne 2024) | Drum diameter, used for the line length |

## Mounting and maintenance notes (QS manual)

- Keep the phase wire outlet pointing **down** so water and dirt don't collect at the axle (p. 6–8).
- Don't seal the wire outlet with glue: the motor "breathes" through it when it heats up and cools down (p. 8).
- Don't carry the motor by the phase wires; it affects waterproofing (p. 5).
- Oil seal: check regularly; QS suggests replacing it after 18–24 months of use (p. 11). Status on this winch: *to be completed*.

## To be completed

- [ ] KV, rated/max phase current, rated torque, voltage range, max rpm, weight (QS spec sheet)
- [ ] Serial number, purchase date, exact order specification
- [ ] Heating test to confirm the KTY83-122 temperature sensor ([WINCH-17](../../features/WINCH-17-hall-sensors-foc.md))
- [ ] Condition of the spare sensor set 2 (original QS plug, unchecked)
- [ ] Pinout of the VESC sensor port vs. plug colours
- [ ] Oil seal status

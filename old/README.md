# Archive: retired code and docs

Nothing in this folder is used on the winch any more. It is kept for reference only and is **not maintained**.
Do not flash anything from here without checking it against the current transmitter/receiver code first.

## Superseded VESC config backups (`vesc-configs/`, archived September 2026, WINCH-17)

The current VESC configuration is in [../vesc/](../vesc/readme.md) (`260930_*.xml`: FOC hall mode, KTY83/122 temperature sensor). **Do not write these old configs to the VESC.**

| File | What it is |
|------|------------|
| `vesc-configs/240601_motor_config.xml` | Etienne, June 2024: motor config of this winch, **sensorless**, temperature sensor type NTC 10k (wrong for this motor; caused `FAULT_CODE_OVER_TEMP_MOTOR` with the sensor cable connected). |
| `vesc-configs/240601_app_config.xml` | Etienne, June 2024: app config (identical to the current one). |
| `vesc-configs/vesc_motor_config_12kw_260_V4.xml` | Robert Zach, 2022: QS 12 kW 260 V4, hall mode (his motor). |
| `vesc-configs/vesc_motor_config_12kw_273.xml` | Robert Zach, 2022: QS 12 kW 273, hall mode. |
| `vesc-configs/vesc_app_config.xml` | Robert Zach, 2022: app config (different PPM pulse range). |

## Cockpit monitor (retired September 2026, WINCH-04)

A monitor on the pilot's cockpit showed winch values (pull, line length, duty cycle) and could change settings.
It was lost in flight once, and it added complexity without really being needed, so it won't be rebuilt.

| Path | What it is |
|------|------------|
| `monitor-ESP-NOW/` | LilyGO T-Display S3 monitor, talks to the transmitter via ESP-NOW (WiFi). Could also set `myMaxPull` with a potentiometer and control relay / line cutter. `mac-address.cpp` is a separate helper sketch to read a board's MAC address. |
| `monitor-LoRa/` | Earlier variant: an additional TTGO LoRa32 board that listens to the LoRa traffic and shows the values. |
| `doc/t-display-s3-pinout.jpg` | Pinout of the T-Display S3. |

The matching ESP-NOW code in `transmitter/transmitter.ino` is **commented out** (four blocks marked `[WINCH-04]`),
not deleted. Note that the ESP-NOW message struct in the transmitter no longer matches the latest monitor code here
(the `loraConnect` field was dropped in WINCH-02).

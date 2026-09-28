# Archive: retired code and docs

Nothing in this folder is used on the winch any more. It is kept for reference only and is **not maintained**.
Do not flash anything from here without checking it against the current transmitter/receiver code first.

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

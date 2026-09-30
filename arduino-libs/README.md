# Arduino libraries (pinned versions)

Copies of the exact library versions used to build `transmitter/` and `receiver/`. They are kept in the repo so the winch firmware can always be rebuilt with the same code, even if a library changes or disappears upstream. See [WINCH-20](../features/WINCH-20-pin-toolchain-libraries.md).

**Do not update these libraries casually.** Update one library at a time, as its own feature, with a bench test (for example, a new Button2 version could change click, double-click or long-press timing, which drives the pull state machine).

## Versions

| Library | Version | Used by | Source | License |
|---------|---------|---------|--------|---------|
| [Button2](Button2/) | 2.3.2 | transmitter | https://github.com/LennartHennigs/Button2 | MIT |
| [LoRa](LoRa/) | 0.8.0 | transmitter, receiver | https://github.com/sandeepmistry/arduino-LoRa | MIT |
| [VescUart](VescUart/) | 1.0.1 | receiver | https://github.com/SolidGeek/VescUart | GPL-3.0 |
| [ESP8266 and ESP32 OLED driver for SSD1306 displays](ESP8266_and_ESP32_OLED_driver_for_SSD1306_displays/) | 4.5.0 | transmitter, receiver | https://github.com/ThingPulse/esp8266-oled-ssd1306 | MIT |
| [Pangodream_18650_CL](Pangodream_18650_CL/) | 1.0.1 | transmitter, receiver | https://github.com/pangodream/18650CL | MIT |

Each folder is an unmodified copy of the installed library, including its license file. Taken from Etienne's Arduino library folder (installed February to April 2024) on 2026-09-30.

The rest of the toolchain can't be stored here and is pinned by version:

| Part | Version |
|------|---------|
| Arduino IDE | 2.3.2 (the old IDE 1.8.x was removed on 2026-09-30; don't use it) |
| Board package "esp32 by Espressif Systems" | **2.0.15** (install exactly this version in the Boards Manager; 3.x has breaking changes) |
| Board | TTGO LoRa32-OLED (`esp32:esp32:ttgo-lora32`) |

## Status

- 2026-09-30: both sketches (state of WINCH-05/06/07) **compile** against exactly these copies (arduino-cli 0.35.3 from Arduino IDE 2.3.2, esp32 core 2.0.15).
- Bench and field tests with firmware built from these versions: *to be completed* after the WINCH-05/06/07 flash. The firmware on the winch before that was possibly built on another machine with unknown library versions (see WINCH-02).

## How to use them

**Arduino IDE:** copy the five library folders from `arduino-libs/` into your Arduino library folder (Windows: `Documents\Arduino\libraries\`), replacing other versions of the same libraries. Then open the sketch, select the board and compile as usual. In the IDE, *Tools → Manage Libraries* should show the versions from the table above.

**arduino-cli** (used by Claude for compile checks, no need to copy anything):

```
arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 --libraries arduino-libs transmitter
arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 --libraries arduino-libs receiver
```

ESP32Servo is no longer needed (line cutter removed in WINCH-05).

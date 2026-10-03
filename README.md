# Remote Controlled Electric Paraglider & Hangglider Winch

  - https://www.youtube.com/shorts/iNt1cCAZv0I (outdated video, will need to make a new one)

This is not a step by step guide of how to build your own electrical paraglider and hangglider winch, but it contains
most resources needed to build your own, including links to the most important components
- see doc/winch-schema.jpg for a simplified overview of all components needed
- doc/parts-list.md contains a list of the main parts and where to purchase them

This is not a finished project, rather a "work in progress". As of Sept. '26 I am currently improving some things, cleaning up,
changing the controller's case to a 3D-printed version, added a Daly BMS to the Battery. More on this later.

In the video above you can see the winch hanging on a steel post. Hauling it around
(getting into the trunk of my car, getting it out again, carrying) was extremely stressful,
since the winch is quite heavy, so I spent the winter of '23/'24 to move the winch onto a bike trailer.
The winch was able to rotate around it's Z-Axis, and fitted with the same arm to guide the line upwards
during the winch process. In Summer '24 this proved to not be ideal. A friend ( Bernd ) developed
a compact "Azimuth System", which is supposed to guide the line up and down and sideways during the winch-process.

During the winter 24/25 the rotary support and the arm were removed, a new frame and the azimuth system was built
and added to the system. I have done since then multiple step tows quite successfully and am really happy with the system.

As of today (July 25), I plan to keep this repo updated

# ewinch_remote_controller
 transmitter and receiver code for remote controlling a paragliding winch
 Based on LILYGO® TTGO ESP32-Paxcounter LoRa32 V2.1 1.6 Version 915MHZ LoRa ESP-32 OLED
 (http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1271&FId=t3:50060:3) 

Note: The 915MHz Version can transmit/receive in 868MHz and 915MHz, the desired frequency is defined in the code (transmitter.ino & receiver.ino)
  
Sketches: `transmitter/transmitter.ino` (handheld remote) and `receiver/receiver.ino` (winch, together with its helper
`LiPoCheck.cpp/.h`). Open the `.ino` in the Arduino IDE, compile and flash. The transmitter is the version flashed on my
remote (synced Sept '26: ID 3, `myMaxPull = 95`), except that the ESP-NOW monitor code is now commented out.

 receiver uses PPM (Pulse Position Modulation) for driving the winch and (optional) UART to read additional information (line length, battery %, dutycycle)
 VESC UART communication depends on https://github.com/SolidGeek/VescUart/ - Note: Line length seems to not be correctly transmitted, falls short by a factor of ~0,7
 
## To use Arduino IDE with the Lilygo TTGO ESP32 Paxcounter LoRa32
- In Arduino IDE open File > Preferences
- in additional boards manager URLS field copy: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- click OK
- Go to Tools > Board > Boards Manager
- In Boards Manager Search for ESP32 and install **version 2.0.15** of “ESP32 by Espressif Systems“ (pick the version in the drop-down; 3.x is not tested with this code)
- Go To Tools > Board > ESP32 and select the TTGO LoRa32-OLED Board

## Install the Arduino libraries
Use the pinned library versions from [arduino-libs/](arduino-libs/README.md): copy the folders into your Arduino library folder (Windows: `Documents\Arduino\libraries\`). Don't update them casually, see the README there.
- [18650CL](https://github.com/pangodream/18650CL) 1.0.1
- [Button2](https://github.com/LennartHennigs/Button2) 2.3.2
- [VescUart](https://github.com/SolidGeek/VescUart) 1.0.1
- [OLED-SSD1306](https://github.com/ThingPulse/esp8266-oled-ssd1306) 4.5.0
- [LoRa](https://github.com/sandeepmistry/arduino-LoRa) 0.8.0

## PIN Setup Receiver:
IO 13 (PWM_PIN_OUT) // connect to PPM Port "Servo" on Vesc

IO 14 (VESC_RX)   //connect to COMM Port "TX" on Vesc

IO 2 (VESC_TX)   //connect to COMM Port "RX" on Vesc

IO 12 (Relay Signal) (for the VESC cooling fan) // connect red wire to 5V, black wire to GND and white cable (signal) to Pin 12. Wire the VESC cooling fan through the relay module. The receiver switches the fan automatically, see "Cooling fan" below. If your relay module is active-low, set `RELAY_ACTIVE_HIGH` to `false` in `receiver.ino`. Wire colours on this winch: see the table below.

### Receiver wiring harness (this winch)
Wire colours as they leave the TTGO LoRa board, recorded by Etienne on 2026-10-03. Two 6-pin plugs: the **female** plug goes only to the fan relay (doubled / free pins leave room for a later function), the **male** plug goes to the VESC. The main supply uses the small 2-pin connector that came with the board (on the transmitter the same connector goes to the 18650 cell).

| Plug | Board pin | Wire colour | Function |
|------|-----------|-------------|----------|
| 6-pin **female** | IO15 | yellow | currently not used (the relay signal was wrongly on IO15 until 2026-10-02) |
| 6-pin **female** | IO12 | white | relay signal (cooling fan) |
| 6-pin **female** | GND | green & black | ground |
| 6-pin **female** | +5V | red & blue | 5 V supply |
| 6-pin **male** | IO2 | white | UART TX → VESC COMM RX (orange wire on the pre-made COMM plug) |
| 6-pin **male** | IO13 | yellow | PPM → VESC "Servo" port |
| 6-pin **male** | IO14 | blue | UART RX ← VESC COMM TX (green wire on the VESC side) |
| 6-pin **male** | + (main supply) | red | board supply from the VESC, via the small 2-pin battery connector on the TTGO board |
| 6-pin **male** | − (main supply) | black | board ground from the VESC, via the same 2-pin connector |

Colours of pre-made cables differ: go by the signal, not by the colour.

## PIN Setup Transmitter:
IO 15 (BUTTON_UP) //together with GND connect with push button for UP Command

IO 12 (BUTTON_DOWN ) //together with GND connect with a push button for STOP/BRAKE Command

A third button (IO 14) was used for the fan relay and the line cutter. It has no function any more.

## Cockpit monitor (retired)
I lost the cockpit monitor (LilyGO T-Display S3) in mid flight, and it added extra tech to take care of,
so I won't rebuild it. Its code and docs are archived in [old/](old/README.md). The matching ESP-NOW code in the
transmitter is commented out.

# VESC
VESC is the Open Source Electronic Speed Controler developed by Benjamin Vedder ( **V**edder **E**lectronic **S**peed **C**ontroller)
Topic has been moved here: [vesc/readme.md](vesc/readme.md). Hands-on guide for VESC Tool (faults, realtime data, settings, hall sensors): [vesc/vesc-tool-guide.md](vesc/vesc-tool-guide.md).

## Cooling fan
The VESC cooling fan is switched by a relay on receiver IO 12. The receiver decides by itself, there is no button for it:
- Fan **ON** as soon as a pull state (state 1 or higher) is active, including the failsafe default pull.
- Fan **OFF** 120 s after the last pull state (`FAN_RUN_ON_MS` in `receiver.ino`). The run-on lets the VESC cool down and keeps the fan from switching on and off during step tows.
- After power-up (soft brake) the fan stays off.
- After a release the remote usually stays in state 1 or 2 while AutoStop holds the drum, so the fan keeps running (the receiver does not know about the VESC AutoStop). It switches off 120 s after the remote goes to brake, or about 140 s after the remote is switched off (20 s failsafe defaultPull, then soft brake, then 120 s run-on).
- The receiver OLED shows "Fan ON", "Fan ON (off in … s)" during the run-on, or "Fan OFF".

The emergency line cutter was dropped (WINCH-05).

# Battery
16P10S Battery using 160 Lithium Ion 21700 cells with 4.000 mAh each.
 - 3,7V nominal per cell, max 4,2V, min 2,5V
 - ~60V total, max 67,2V, min 40V
 - 4Ah per cell = 40Ah total
 - 2,4 kWh

# usage:
Pull values below are for `myMaxPull = 95` in the transmitter (set it to roughly the pilot's take-off weight).
States 2-5 scale with it: prePull 18 %, takeOffPull 55 %, fullPull 80 %, strongPull 100 %.
defaultPull (7kg) and the brakes (-7kg / -20kg) are fixed values.

- A) prepare:
  1 - turn the VESC and receiver on
  2 - MAKE SURE to have the Potentiometer connected to ADC turned fully left (in my setup! need to measure whether this is open or closed :-) )
      If the Poti is rotated to the right, either partially or fully, the winch will not operate with the predefined Pull Torque!
  3 - pull the line out to the desired length (the VESC measures the line length that is being unwound, needed for the autostop to work).
      **Always pull the line out with the soft brake active (state -1, the start state), never in neutral (state 0).**

  > ⚠️ **WARNING - pull the line out with soft brake only!**
  > In neutral nothing holds the drum. When you walk faster and then stop, the drum keeps turning and unwinds
  > line right at the drum (overrun). On launch the loose turns cause a line wrap. **This has already happened
  > once:** the wrap destroyed the line and the (3D-printed) gear of the winch. The soft brake keeps the line
  > under light tension so the drum stops when you stop.
  > (The winding gears have since been replaced: first printed in plastic to check function and fit,
  > now laser-sintered steel for permanent use.)
  4 - go through your pre-flight preparations and clip in
- B) launch:
  1 - switch to defaultPull (7kg pull value) and prePull (to tighten the line (~17kg pull value) to assist you to launch the glider
  2 - go to takeOffPull (~52kg pull value) to assist you with launching the glider and gently getting into the air with a slight pull towards a safety margin of 15-30m height
  4 - click "Up" Button to increase Pull
- C) Step Towing:
  1 - switch to defaultPull (7kg pull value) before you turn away from winch to fly back to launch site
  2 - go to prePull (~17kg pull value) during turn towards next step (towards winch) to avoid line sag
  3 - after successful turn, go to fullPull again
 
- C) Release
  Go To defaultPull (7kg pull value) before you release. After releasing, rewind the line with LOW pull only:
  **maximum state 2 (prePull, ~17kg) - never higher.**
  The AutoStop feature (modified VESC Firmware is required, see vesc/vesc_ppm_auto_stop.patch) brakes the drum
  when approx. 15m of line are left - but only reliably at low rewind speed.

  > ⚠️ **WARNING - rewind with max. state 2 only!**
  > AutoStop only works reliably if the line is rewound with little pull (max. state 2 / prePull) after releasing.
  > With more pull, the brake is not strong enough to stop the drum in time: the carabiner is pulled into the
  > azimuth system and destroys it, or the line snaps. **This has already happened once** - the line broke and
  > tangled and the whole winch had to be rebuilt. (An earlier version of this README wrongly said to rewind
  > with fullPull.)

  > ⚠️ **WARNING - do NOT switch the remote off until the line is fully rewound and AutoStop has stopped the drum!**
  > The receiver does not know about the VESC AutoStop. If the remote goes off while the line is still being
  > rewound, the receiver failsafe keeps only defaultPull for 20 s and then switches to soft brake: the drum stops
  > and the rest of the line falls onto the towing track and stays there. **This has already happened once.**
  > Keep the remote on (state 1 or 2) until AutoStop has stopped the drum, and only then switch it off.
 
- D) Neutral
  You can get to neutral state only if you are in Brake Mode (-7kg), Double Press the ButtonDown to activate it.
  **Do not use neutral to pull the line out** (see the warning in A: the drum overruns and the line wraps).
  Use the soft brake instead. (An earlier version of this README recommended neutral for pulling the line out.)

- E) Rewinding the Cable
  If something does not go as expected (during one flight, I turned off the remote too soon after release: the failsafe stopped the winch, as designed, and the line fell on the ground, see the warning in C) Release):
  - use the Potentiometer, which should be on the far left position, to gently rotate it towards the right. The motor will start to rewind the line, regardless of the measured distance.

## Several pilots / several remotes
- **One pilot:** flash `transmitter/transmitter.ino` onto one remote with any ID from 1 to 15 (`myID`) and the pilot's take-off weight as `myMaxPull`. That's all you need to tow yourself.
- **Several pilots:** prepare one remote per pilot. For each remote, change `myID` (unique, 1-15) and `myMaxPull` (that pilot's take-off weight) in the code and flash it. Put a label with ID and max pull on each remote.
- **Who controls the winch:** the receiver follows only one remote at a time. The remote that is sending keeps control as long as it is switched on (it sends every 400 ms). Another remote can only take over after the active one has been **silent for 5 s** (switched off or out of range).
- **Admin remote (ID 0):** takes over immediately, at any time. When switched on, it listens for 4 s, starts in the state the winch is currently in, and from then on its buttons control the winch. Build it so it can't be confused with the others (Etienne's admin remote has a **red case**).
- ⚠️ When the admin remote is switched off again, the receiver keeps the last state; if it was a pull state, it goes to failsafe after 1.5 s (default pull, soft brake after 20 s). After 5 s **any other remote that is still switched on takes over with whatever state it is sending**. Before switching the admin remote off, make sure the other remotes are off or in a safe state (soft brake).
- All remotes and the receiver must run the same code version (same LoRa message format), see [features/INDEX.md](features/INDEX.md).

# Roadmap / ToDo
Planned changes, open bugs and ideas are tracked in [features/INDEX.md](features/INDEX.md), one file per feature.

# Notes
from Robert's "Issue Section"
- calibrate PWM Settings for pull Values: Depending on your motor (power, KV value, diameter, etc.) you need to scale the resulting kg pull on your line.
I think the best way is to adapt the "Motor Current Max" and "Motor Current Max Brake" value in the Vesc Motor Settings (General -> Current).
You could also adapt the "Pulselength Start" and "Pulselength End" in the Vesc App settings (PPM -> Mapping) to reach the same goal.
I used a suitcase scale to measure the real pull on the line.
With my QS260 hub motor I have around ~3,6A/kg pull.
- Motor amount of Poles: 32

# Big "Thank You"

A big "Thank You" goes to Robert Zach, from whom I copied this project (and forked his code).
https://github.com/robertzach/ewinch_remote_controller
Many Thanks to Bernd Otterpohl for designing the Azimuth System and making the files available
Many Thanks to Nico Stucke and Hans Werner Stucke for making the frame and the actual azimuth system.

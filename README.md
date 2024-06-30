# Remote Controlled Electric Paraglider Winch

  - https://www.youtube.com/shorts/iNt1cCAZv0I

This is not a step by step guide of how to build your own electrical paraglider winch, but it contains
most resources needed to build your own, including links to the most important components
- see doc/winch-schema.jpg for a simplified overview of all components needed
- doc/parts-list.md contains a list of the main parts and where to purchase it

This is not a finished project, rather a "work in progress" (as of May '24)

In the video above you can see the winch hanging on a steel post. Hauling it around
(getting into the trunk of my car, getting it out again, carrying) was extremely stressful,
since the winch is quite heavy, so I spent the winter of '23/'24 to move the winch onto a bike trailer.

As of today, I plain to keep this repo updated

# ewinch_remote_controller
 transmitter and receiver code for remote controlling a paragliding winch
 Based on LILYGO® TTGO ESP32-Paxcounter LoRa32 V2.1 1.6 Version 915MHZ LoRa ESP-32 OLED
 (http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1271&FId=t3:50060:3) 

Note: The 915MHz Version can transmit/receive in 868MHz and 915MHz, the desired frequency is defined in the code (transmitter.ino, monitor.ino & receiver.ino)
  
 receiver uses PPM (Pulse Position Modulation) for driving the winch and (optional) UART to read additional information (line length, battery %, dutycycle)
 VESC UART communication depends on https://github.com/SolidGeek/VescUart/ - Note: Line length seems to not be correctly transmitted, falls short by a factor of ~0,7
 
## To use Arduino IDE with the Lilygo TTGO ESP32 Paxcounter LoRa32 / (and Lilygo T-Display S3)
- In Arduino IDE open File > Preferences
- in additional boards manager URLS field copy: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- click OK
- go to Go to Tools > Board > Boards Manager
- In Boards Manager Search for ESP32 and press install button for the “ESP32 by Espressif Systems“
- Go To Tools > Board > ESP32 and select the TTGO LoRa32-OLED Board

## Install the following Arduino libraries:
- [18650CL](https://github.com/pangodream/18650CL)
- [Button2](https://github.com/LennartHennigs/Button2)
- [VescUart](https://github.com/SolidGeek/VescUart)
- [OLED-SSD1306](https://github.com/ThingPulse/esp8266-oled-ssd1306)
- [Servo](https://www.arduino.cc/reference/en/libraries/esp32servo/)
- [TFT_eSPI](https://www.arduino.cc/reference/en/libraries/tft_espi/) / If you want to use the Lilygo T-Display S3 as a monitor

## Moving to Visual Studio Code and PlatformIO

I am moving to Visual Studio Code and PlatformIO to introduce Version control with Github and Git.
The main difference is that with VS Code and PlatformIO the main program files don't have the *.ino file extension
but *.cpp. If you want to compile and upload the code to the ESP32 boards just download all files and rename
them from *.cpp to *.ino. There are some other challenges with Visual Studio Code and PlatformIO that may require
to move the code for each platform to it's own repository, i.e. "transmitter", "receiver" and "monitor"

## PIN Setup Receiver:
IO 13 (PWM_PIN_OUT) // connect to PPM Port "Servo" on Vesc

IO 14 (VESC_RX)   //connect to COMM Port "TX" on Vesc

IO 2 (VESC_TX)   //connect to COMM Port "RX" on Vesc

IO 15 (Servo Signal) // connect red wire to 5V, black or brown wire to GND and yellow or white cable to Pin 15. Servo is in neutral state by default // info to self: white cable to "S", black/green to "-", red/blue to "+"

IO 12 (Relay Signal) // connect red wire to 5V, black wire to GND and white cable (signal) to Pin 12. Wire your Warning Light and VESC Cooling Fan through the relay module. Relay is off by default, will be turned on once the Remote/Transmitter is turned on. // note to self: yellow to yellow (signal), red to red/blue, brown to black/green.

## PIN Setup Transmitter:
IO 15 (BUTTON_UP) //together with GND connect with push button for UP Command

IO 12 (BUTTON_DOWN ) //together with GND connect with a push button for STOP/BRAKE Command

IO 14 (BUTTON_THREE ) // additional /optional Button for Relay and Servo Control. Click once to deactivate Relay (Fan and Warning light), Click again to turn on again. Long Click to trigger the Emergency Line Cutter (Servo). Doubleclick to move Servo to Neutral Position again. By default, the Servo is in neutral position, and Relay is off. Turning on the Remote will turn on the Relay automatically.

## Monitor with LilyGo T-Display
https://www.lilygo.cc/products/t-display-s3 
- a monitor on your paraglider cockpit to have winch values in eyesight
- a control unit for the emergency line cutter and fan / warning light
- a control unit to change the settings for "maxPull" with the help of a potentiometer

## Pin Setup Monitor
IO 16 Connect a Potentiometer to PIN 16 (dont forget GND and +3,3V)

IO 0 Button A - Relay and cooling fan are on by default. Press Button A once to turn off, and on again

IO 14 Button B - Button B should be a large and easily reachable button on your cockpit. This triggers the emergency line cutter. A double press resets the Servo / Line Cutter back to neutral/ready position

IO 12 Button C - Press Button C once to enter "settings mode". You can now turn the potentiometer to set the Max Pull Value to the transmitter (set Value according to Pilot's Take-Off weight) Press Button once more to exit settings mode and confirm the selected value. It is then sent
to the transmitter over Wifi via ESP-Now Protocol.

# VESC
VESC is the Open Source Electronic Speed Controler developed by Benjamin Vedder ( **V**edder **E**lectronic **S**peed **C**ontroller)
Topic has been moved here: https://github.com/looxis-renaud/ewinch_remote_controller/tree/main/vesc#readme

## Remote Control of cooling fan and warning light (DHV Regulations)
The transmitter and receiver code now supports a Relay that is automatically turned on when the
remote is switched on. The relay can control the cooling fan of the VESC and a warning light (as required by DHV regulations)
A a single click on a third button on the remote turns the Relay (Fan and warning light) off - usefull when you launched yourself
and want to fly away and leave the winch behind.

## Remote Control of an Emergency Line Cutter (DHV Regulations)
The transmitter and receiver code now supports the connection of a Servo (on PIN 15 on Receiver).
The servo rotates 90 degrees when the third button on the transmitter is "long pressed" (one full second).
This servo should be attached to an emergency line cutter, that cuts the dyneema line in an emergency.
Note: Activating the line cutter also triggers the full brake (-20kg)

# Battery
16P10S Battery using 160 Lithium Ion 21700 cells with 4.000 mAh each.
 - 3,7V nominal per cell, max 4,2V, min 2,5V
 - ~60V total, max 67,2V, min 40V
 - 4Ah per cell = 40Ah total
 - 2,4 kWh

# usage:
- A) prepare:
  1 - turn the VESC and receiver on
  2 - pull the line out to the desired length (the VESC measures the line length that is being unwound, needed for the autostop to work)
  3 - go through your pre-flight preparations and clip in
- B) launch:
  1 - switch to defaultPull (7kg pull value) and prePull (to tighten the line (~13kg pull value) to assist you to launch the glider
  2 - go to takeOffPull (~40kg pull value) to assist you with launching the glider and gently getting into the air with a slight pull towards a safety margin of 15-30m height
  4 - click "Up" Button to increase Pull
- C) Step Towing:
  1 - switch to defaultPull (7kg pull value) before you turn away from winch to fly back to launch site
  2 - go to prePull (~15kg pull value) during turn towards next step (towards winch) to avoid line sag
  3 - after successful turn, go to fullPull again
 
- C) Release
  Go To defaultPull (7kg pull value) before you release
  The winch will autorewind the line IF AutoStop feature is enabled (modified VESC Firmware is required, see vesc/vesc_ppm_auto_stop.patch)
 
- D) Neutral
  You can get to neutral state only if you are in Brake Mode (-7kg), Double Press the ButtonDown to activate it.
 
# ToDo
- set up some sort of encryption or password for a secure connection between transmitter and receiver
- set up some way to calibrate PWM Settings and Pull Values.
- add Bernd's line cutter
- improve mechanics and mounting on the bike trailer

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

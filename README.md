# ewinch_remote_controller
 transmitter and receiver code for remote controlling a paragliding winch
 Based on LILYGO® TTGO ESP32-Paxcounter LoRa32 V2.1 1.6 Version 915MHZ LoRa ESP-32 OLED
 (http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1271&FId=t3:50060:3) 

Note: The 915MHz Version apparently can transmit/receive in 868MHz and 915MHz, the desired frequency is defined in the code (transmitter.ino, monitor.ino & receiver.ino)
 
 see https://www.youtube.com/watch?v=5IkagHkxbxY

 receiver uses PPM (Pulse Position Modulation) for driving the winch and (optional) UART to read additional information (line length, battery %, dutycycle)
 VESC UART communication depends on https://github.com/SolidGeek/VescUart/
 
## To use Arduino IDE with the Lilygo TTGO ESP32 Paxcounter LoRa32
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

## PIN Setup Receiver:
IO 13 (PWM_PIN_OUT) // connect to PPM Port "Servo" on Vesc

IO 14 (VESC_RX)   //connect to COMM Port "TX" on Vesc

IO 2 (VESC_TX)   //connect to COMM Port "RX" on Vesc

IO 15 (Servo Signal) // connect red wire to 5V, black or brown wire to GND and yellow or white cable to Pin 15

ToDo: Connect Relay for Fan/Warning Light Control


## PIN Setup Transmitter:
IO 15 (BUTTON_UP) //together with GND connect with push button for UP Command

IO 12 (BUTTON_DOWN ) //together with GND connect with a push button for STOP/BRAKE Command

## Line auto stop in VESC
Line auto stop can be implemented within VESC with vesc_ppm_auto_stop.patch

For this to work properly, either connect a Potentiometer to ADC2 and GND to manually control the winch. E.g. To wind up the last meters of the line when finishing. Or to manually set a tension when used as a rewind winch. Note that the potentiometer only reduces tension/speed of the motor when it is running one of the pull programs as controlled via the transmitter!

If you do not install a Potentionmeter, connect ADC2 to GND.

## Remote Control of cooling fan and warning light (DHV Regulations)
The transmitter and receiver code now supports a Relay that is automatically turned on when the
remote is switched on. The relay can control the cooling fan of the VESC and a warning light (as required by DHV regulations)
A a single click on a third button on the remote turns the Relay (Fan and warning light) off - usefull when you launched yourself
and want to fly away and leave the winch behind.
TODO - the relay itself hasn't been implemented yet, working on it

## Remote Control of an Emergency Line Cutter (DHV Regulations)
The transmitter and receiver code now supports the connection of a Servo (on PIN 15 on Receiver).
The servo rotates 90 degrees when the third button on the transmitter is "long pressed" (one full second).
This servo should be attached to an emergency line cutter, that cuts the dyneema line in an emergency.
TODO - Design and build a lightweight and efficient line cutter! Bernd O., this one is on you :-)


## Default Config for VESC
Default VESC app config is vesc_app_config.xml
Default Motor config is vesc_motor_config_12kw_260_V4.xml or vesc_motor_config_12kw_273.xml
**PLEASE NOTE:** don't just take the standard motor config and upload to your VESC. Take it as an example only.
Make sure to run the **"Setup Motor FOC"** wizard for the VESC tool to properly detect internal resistances and other values.

## usage:
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
 
 ## UNCLEAR
Ask Robert:
- instruction for PPM Settings says "static int myMaxPull = 85;  // 0 - 127 [kg], must be scaled with VESC ppm settings" how to set or calibrate this ?

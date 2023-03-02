# ewinch_remote_controller
 transmitter and receiver code for remote controlling a paragliding winch
 Based on LILYGO® TTGO ESP32-Paxcounter LoRa32 V2.1 1.6 Version 915MHZ LoRa ESP-32 OLED
 (http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1271&FId=t3:50060:3) 

Note: The 915MHz Version apparently can transmit/receive in 868MHz and 915MHz, the desired frequency is defined in the code (transmitter.ino, monitor.ino & receiver.ino)
 
 see https://www.youtube.com/watch?v=5IkagHkxbxY

 receiver uses PPM (Pulse Position Modulation) for driving the winch and (optional) UART to read additional information (line length, battery %, dutycycle)
 VESC UART communication depends on https://github.com/SolidGeek/VescUart/
 

## Install the following Arduino libraries:
- [18650CL](https://github.com/pangodream/18650CL)
- [Button2](https://github.com/LennartHennigs/Button2)
- [VescUart](https://github.com/SolidGeek/VescUart)
- [OLED-SSD1306](https://github.com/ThingPulse/esp8266-oled-ssd1306) //I could not get the display work with Robert Zach's Code, so I updated some of the code. more of this later

## PIN Setup Receiver:
IO 13 (PWM_PIN_OUT) // connect to PPM Port "Servo" on Vesc

IO 14 (VESC_RX)   //connect to COMM Port "TX" on Vesc

IO 2 (VESC_TX)   //connect to COMM Port "RX" on Vesc


## PIN Setup Transmitter:
IO 15 (BUTTON_UP) //together with GND connect with push button for UP Command

IO 12 (BUTTON_DOWN ) //together with GND connect with a push button for STOP/BRAKE Command

## Line auto stop in VESC
Line auto stop can be implemented within VESC with vesc_ppm_auto_stop.patch

Default VESC app config is vesc_app_config.xml

## usage:
- A) launch: ...
- B) Step Towing:
  1 - switch to defaultPull (7kg pull value) before you turn away from winch to fly back to launch site
  2 - go to prePull (~13kg pull value) during turn towards next step (towards winch) to avoid line sag
  3 - after successful turn, go to fullPull again
 
- C) Release
  Go To defaultPull (7kg pull value) before you release
  The winch will autorewind the line IF AutoStop feature is enabled (modified VESC Firmware is required, see vesc/vesc_ppm_auto_stop.patch)
 
- D) Neutral
  You can get to neutral state only if you are in Brake Mode (-7kg), Double Press the ButtonDown to activate it.
 
 ## UNCLEAR
Ask Robert:
- which mode is needed to pull the line from winch to start ? "neutral" ?
- why is neutral NOT default ?
- instruction for PPM Settings says "static int myMaxPull = 85;  // 0 - 127 [kg], must be scaled with VESC ppm settings" how to set or calibrate this ?

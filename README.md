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

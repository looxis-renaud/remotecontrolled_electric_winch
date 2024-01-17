/* TRANSMITTER
 * sends current state (pull value)
 * receives acknowlegement with current parameters
 * communication is locked to a specific transmitter for 5 seconds after his last message
 * admin ID 0 can always take over communication
 * Added support for a third button to control a Relay and a Servo.
 * The Relay can turn the VESC Cooling Fan and Warning Light (DHV Regulations) on and off
 * The Servo can trigger an Emergency Line Cutter (DHV Regulations)
 */

static int myID = 8;    // set to your desired transmitter id, "0" is for admin 1 - 15 is for additional transmitters [unique number from 1 - 15]
static int myMaxPull = 85;  // 0 - 127 [kg], must be scaled with VESC ppm settings

#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <LoRa.h>

static int myID = 8;    // set to your desired transmitter id, "0" is for admin 1 - 15 is for additional transmitters [unique number from 1 - 15]
static int myMaxPull = 85;  // 0 - 127 [kg], must be scaled with VESC ppm settings

#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <LoRa.h>

// Include the correct display library for a connection via I2C using Wire include
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

// Initialize the OLED display using Wire library
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL - SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  868E6  //frequency in Hz (433E6, 868E6, 915E6) 

int rssi = 0;
float snr = 0;
String packSize = "--";
String packet ;

#include <rom/rtc.h>

#include "Arduino.h"
#include <Button2.h>

// battery measurement
//#define CONV_FACTOR 1.7
//#define READS 20
Pangodream_18650_CL BL(35); // pin 34 old / 35 new v2.1 hw

// Buttons for state machine control
#define BUTTON_UP  15 // up
#define BUTTON_DOWN  12 // down

Button2 btnUp = Button2(BUTTON_UP);
Button2 btnDown = Button2(BUTTON_DOWN);

/* optional third button for controlling a Servo and a Relay.
* The Servo could trigger a line cutter in an emergency.
* The Relay could disable the fan and warning light
* after completing a successful self-launch
*/

#define BUTTON_THREE  14 // Third button on pin14, 
Button2 btnThree = Button2(BUTTON_THREE);

static int loopStep = 0;
bool toogleSlow = true;
int8_t targetPull = 0;   // pull value range from -127 to 127
int currentPull = 0;          // current active pull on vesc
bool stateChanged = false;
int currentState = -1;   // status to start with: -2 = hard brake, -1 = soft brake, 0 = no pull/no brake, 1 = default pull (~3kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
int hardBrake = -20;  //in kg - status -2 this status is 20kg brake - 
int softBrake = -7;  //in kg - status -1 this status is 7kg brake - activated with a long stop press on "buttonDown"
int defaultPull = 7;  //in kg - status 1 this will always be activated when the "brake-button" (buttonDown) is pressed (unless a long press, which activates the "soft-break" status)
int prePullScale = 18;      //in %, calculated in code below: "targetPull = myMaxPull * prePullScale / 100;" - status 2 this is to help you launch your glider up into the air, approx 13kg pull value
int takeOffPullScale = 55;  //in % - status 3 this is to gently and safely take off with a slight pull
int fullPullScale = 80;     //in % - status 4 once you have a safety margin of 15-30m heigt, switch to this status for full pull
int strongPullScale = 100;  //in % - status 5 - extra strong pull - this is equal your value in kg as defined in "myMaxPull" above. should be equal your bodyweight
unsigned long lastStateSwitchMillis = 0;

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;

// Servo and Relay variables
bool servo = false;
bool relay = true;

// #include "common.h" instead of having this as a separate file, let's include it directly - START:
/*
* Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com
* This program is free software : you can redistribute it and / or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
* GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

//send by transmitter
struct LoraTxMessage {
   uint8_t id : 4;              // unique id 1 - 15, id 0 is admin!
   int8_t currentState : 4;    // -2 --> -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
   int8_t pullValue;           // target pull value,  -127 - 0 --> 5 brake, 0 - 127 --> pull
   int8_t pullValueBackup;     // to avoid transmission issues, TODO remove, CRC is enough??
   bool servo = false;         // is supposed to send Servo position for emergency line cutter
   bool relay = true;       // is supposed to turn relay on and off. Fan and warning light will be connected to relay
};

//send by receiver (acknowledgement)
struct LoraRxMessage {
   int8_t pullValue;           // currently active pull value,  -127 - 0 --> 5 brake, 0 - 127 --> pull
   uint8_t tachometer;          // *10 --> in meter
   uint8_t dutyCycleNow;
   uint8_t vescBatteryOrTempMotor : 1 ;  // 0 ==> vescTempMotor , 1 ==> vescBatteryPercentage
   uint8_t vescBatteryOrTempMotorValue  : 7 ;   //0 - 127
};

// END include common.h

struct LoraTxMessage loraTxMessage;
struct LoraRxMessage loraRxMessage;

unsigned long lastTxLoraMessageMillis = 0;    //last message send
unsigned long lastRxLoraMessageMillis = 0;    //last message received
unsigned long previousRxLoraMessageMillis = 0;

unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;

void setup() {
  Serial.begin(115200);
  

  //lora init
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  //LoRa.setSpreadingFactor(10);   // default is 7, 6 - 12
  LoRa.enableCrc();
  //LoRa.setSignalBandwidth(500E3);   //signalBandwidth - signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.

  // display init
  display.init();
  //display.flipScreenVertically();  

  //Serial.println(" Longpress Time: " + String(btnUp.getLongClickTime()) + "ms");
  //Serial.println(" DoubleClick Time: " + String(btnUp.getDoubleClickTime()) + "ms");
  btnUp.setPressedHandler(btnPressed);
  btnDown.setPressedHandler(btnPressed);
  btnDown.setLongClickTime(500);
  btnDown.setLongClickDetectedHandler(btnDownLongClickDetected);
  btnDown.setDoubleClickTime(400);
  btnDown.setDoubleClickHandler(btnDownDoubleClick);
  
  btnThree.setPressedHandler(btnThreePressed);
  btnThree.setDoubleClickTime(400);
  btnThree.setDoubleClickHandler(btnThreeDoubleClick);
  btnThree.setLongClickTime(500);
  btnThree.setLongClickDetectedHandler(btnThreeLongClickDetected);
    
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Transmitter \n");
  display.drawString(0, 0, "Starting Transmitter");

  // admin --> scan for existing transmitter for a few seconds --> start up with his current pull state
  if (myID == 0 ) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "Searching 4s for");
      display.drawString(0, 14, "existing transmitter...");
      display.display();
      lastTxLoraMessageMillis = millis();
      while (millis() < lastTxLoraMessageMillis + 4000) {
          // packet from transmitter
          if (LoRa.parsePacket() >= sizeof(loraTxMessage) ) {
            LoRa.readBytes((uint8_t *)&loraTxMessage, sizeof(loraTxMessage));
            if (loraTxMessage.pullValue == loraTxMessage.pullValueBackup) {
                //found --> read state and exit
                currentState = loraTxMessage.currentState;
                targetPull = loraTxMessage.pullValue;
                Serial.printf("Found existing transmitter, starting up with state: %d: %d \n", currentState, targetPull);
                //exit search loop
                lastTxLoraMessageMillis = millis() - 4000;
            }
          } 
          delay(10);
       }
   }
   // reset to my transmitter id
   loraTxMessage.id = myID;
}

void loop() {

    loopStep++;
  
    // screen
    if (loopStep % 100 == 0) {
      toogleSlow = !toogleSlow;
    }
    if (loopStep % 10 == 0) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);  //10, 16, 24
      if (toogleSlow) {
          display.drawString(0, 0, loraTxMessage.id + String("-B: ") + vescBattery + "%, T: " + vescTempMotor + " C");        
      } else {
          display.drawString(0, 0, loraTxMessage.id + String("-T: ") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");        
      }
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + currentPull + String("kg)"));
      display.drawString(0, 36, String(loraRxMessage.tachometer * 10) + "m| " + String(loraRxMessage.dutyCycleNow) + "%" );
      display.display();
    }
    
    // LoRa data available?
    //==acknowledgement from receiver?
    if (LoRa.parsePacket() == sizeof(loraRxMessage) ) {
        LoRa.readBytes((uint8_t *)&loraRxMessage, sizeof(loraRxMessage));
        currentPull = loraRxMessage.pullValue;
        // vescBatteryPercentage and vescTempMotor are alternated on lora link to reduce packet size
          if (loraRxMessage.vescBatteryOrTempMotor == 1){
            vescBattery = loraRxMessage.vescBatteryOrTempMotorValue;
          } else {
            vescTempMotor = loraRxMessage.vescBatteryOrTempMotorValue;
          }
        previousRxLoraMessageMillis = lastRxLoraMessageMillis;  // remember time of previous paket
        lastRxLoraMessageMillis = millis();
        rssi = LoRa.packetRssi();
        snr = LoRa.packetSnr();
        // Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraRxMessage.pullValue, rssi, snr);
        // Serial.printf("tacho: %d, dutty: %d: \n", loraRxMessage.tachometer * 10, loraRxMessage.dutyCycleNow);
        Serial.printf("Relay: %d\n", relay);
        Serial.printf("Servo: %d\n", servo);
   }

  // if no lora message for more then 1,5s --> show error on screen + acustic
  if (millis() > lastRxLoraMessageMillis + 1500 ) {
        //TODO acustic information
        //TODO  red display
        display.clear();
        display.display();
        // log connection error
       if (millis() > loraErrorMillis + 5000) {
            loraErrorMillis = millis();
            loraErrorCount = loraErrorCount + 1;
       }
  }
  
        // state machine
        // -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
        switch(currentState) {
            case -2:
              targetPull = hardBrake; // -> hard brake
              break;
            case -1:
              targetPull = softBrake; // -> soft brake
              break;
            case 0:
              targetPull = 0; // -> neutral, no pull / no brake
              break;
            case 1: 
              targetPull = defaultPull;   //independent of max pull
              break;
            case 2: 
              targetPull = myMaxPull * prePullScale / 100;
              break;
            case 3:
              targetPull = myMaxPull * takeOffPullScale / 100;
              break;
            case 4:
              targetPull = myMaxPull * fullPullScale / 100;
              break;
            case 5:
              targetPull = myMaxPull * strongPullScale / 100;
              break;
            default: 
              targetPull = softBrake;
              Serial.println("no valid state");
              break;
          }

        delay(10);

        // send Lora message every 400ms  --> three lost packages lead to failsafe on receiver (>1,5s)
        // send immediatly if state has changed
        if (millis() > lastTxLoraMessageMillis + 400 || stateChanged) {
            stateChanged = false;
            loraTxMessage.currentState = currentState;
            loraTxMessage.pullValue = targetPull;
            loraTxMessage.pullValueBackup = targetPull;
            loraTxMessage.servo = servo;
            loraTxMessage.relay = relay;

	    // here we'll send everything in binary format over LoRa:
            if (LoRa.beginPacket()) {
                LoRa.write((uint8_t*)&loraTxMessage, sizeof(loraTxMessage));
                LoRa.endPacket();
                // Serial.printf("sending value %d: \n", targetPull);
                lastTxLoraMessageMillis = millis();  
            } else {
                Serial.println("Lora send busy");
            }
        }

        btnUp.loop();
        btnDown.loop();
      	btnThree.loop();
        delay(10);
}

//code that converts the button presses into action:

void btnPressed(Button2& btn) {
    if (btn == btnUp) {
        Serial.println("+++++ Button Up pressed +++++");
        //do not switch up to fast
        if (millis() > lastStateSwitchMillis + 1000 && currentState < 5) {
          currentState = currentState + 1;
          // skip neutral state to prevent line mess up
          if (currentState == 0 ) {
              currentState = currentState + 1;
          }
          lastStateSwitchMillis = millis();
          stateChanged = true;
        }
    } else if (btn == btnDown) {
        Serial.println("+++++ Button Down pressed +++++");
        if (currentState > 1) {
          currentState = 1;   //default pull
          lastStateSwitchMillis = millis();
          stateChanged = true;
        } else if (currentState > -2 && currentState < 1){
          currentState = currentState - 1;
          lastStateSwitchMillis = millis();
          stateChanged = true;
        }
    }
}

void btnDownLongClickDetected(Button2& btn) {
    Serial.println("+++++ Button Down Looooong pressed +++++");
    currentState = -1;    //brake
    lastStateSwitchMillis = millis();
    stateChanged = true;
	}
void btnDownDoubleClick(Button2& btn) {
  Serial.println("+++++ Double Click on Button Down +++++");
  // only get to neutral state from brake
  if (currentState <= -1) {
    currentState = 0;    // neutral
    lastStateSwitchMillis = millis();
    stateChanged = true;
	}
}
//additional code to handle Button Three (Servo and Relay)
void btnThreePressed(Button2& btn) {
  if (relay == true) {
  Serial.println("+++++ Relay Turned OFFFFFFFFFFFFF +++++");
  relay = false; // turns relay off, which will deactivate Vesc Cooling Fan and Warning Light
  lastStateSwitchMillis = millis();
  stateChanged = true;
  } else {
  Serial.println("+++++ Relay Turned On +++++");
  relay = true; // turns relay on, cooling and warning runs again
  lastStateSwitchMillis = millis();
  stateChanged = true;
  }
 }

void btnThreeDoubleClick(Button2& btn) {
  Serial.println("+++++ DoubleClick on Third Button +++++");
  servo = false; // returns servo to neutral
  lastStateSwitchMillis = millis();
  stateChanged = true;
  }

void btnThreeLongClickDetected(Button2& btn) {
  Serial.println("+++++ LOOOOOOng Click on Third Button +++++");
  servo = true; // use only in emergency, this will trigger a line cutter, yet to be built -> Bernd, deine Aufgabe!
    currentState = -2;    //hard brake, when line is being cut, of course!
    lastStateSwitchMillis = millis();
    stateChanged = true;
  }ire include
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

// Initialize the OLED display using Wire library
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL - SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  868E6  //frequency in Hz (433E6, 868E6, 915E6) 

int rssi = 0;
float snr = 0;
String packSize = "--";
String packet ;

#include <rom/rtc.h>

#include "Arduino.h"
#include <Button2.h>

// battery measurement
//#define CONV_FACTOR 1.7
//#define READS 20
Pangodream_18650_CL BL(35); // pin 34 old / 35 new v2.1 hw

// Buttons for state machine control
#define BUTTON_UP  15 // up
#define BUTTON_DOWN  12 // down, 4 old / 12 new v2.1 hw

Button2 btnUp = Button2(BUTTON_UP);
Button2 btnDown = Button2(BUTTON_DOWN);

static int loopStep = 0;
bool toogleSlow = true;
int8_t targetPull = 0;   // pull value range from -127 to 127
int currentPull = 0;          // current active pull on vesc
bool stateChanged = false;
int currentState = -1;   // status to start with: -2 = hard brake, -1 = soft brake, 0 = no pull/no brake, 1 = default pull (~3kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
int hardBrake = -20;  //in kg - status -2 this status is 20kg brake - 
int softBrake = -7;  //in kg - status -1 this status is 7kg brake - activated with a long stop press on "buttonDown"
int defaultPull = 7;  //in kg - status 1 this will always be activated when the "brake-button" (buttonDown) is pressed (unless a long press, which activates the "soft-break" status)
int prePullScale = 18;      //in %, calculated in code below: "targetPull = myMaxPull * prePullScale / 100;" - status 2 this is to help you launch your glider up into the air, approx 13kg pull value
int takeOffPullScale = 55;  //in % - status 3 this is to gently and safely take off with a slight pull
int fullPullScale = 80;     //in % - status 4 once you have a safety margin of 15-30m heigt, switch to this status for full pull
int strongPullScale = 100;  //in % - status 5 - extra strong pull - this is equal your value in kg as defined in "myMaxPull" above. should be equal your bodyweight
unsigned long lastStateSwitchMillis = 0;

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;

/*
* Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com
* This program is free software : you can redistribute it and / or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
* GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

//send by transmitter
struct LoraTxMessage {
   uint8_t id : 4;              // unique id 1 - 15, id 0 is admin!
   int8_t currentState : 4;    // -2 --> -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
   int8_t pullValue;           // target pull value,  -127 - 0 --> 5 brake, 0 - 127 --> pull
   int8_t pullValueBackup;     // to avoid transmission issues, TODO remove, CRC is enough??
};
//send by receiver (acknowledgement)
struct LoraRxMessage {
   int8_t pullValue;           // currently active pull value,  -127 - 0 --> 5 brake, 0 - 127 --> pull
   uint8_t tachometer;          // *10 --> in meter
   uint8_t dutyCycleNow;
   uint8_t vescBatteryOrTempMotor : 1 ;  // 0 ==> vescTempMotor , 1 ==> vescBatteryPercentage
   uint8_t vescBatteryOrTempMotorValue  : 7 ;   //0 - 127
};

struct LoraTxMessage loraTxMessage;
struct LoraRxMessage loraRxMessage;


unsigned long lastTxLoraMessageMillis = 0;    //last message send
unsigned long lastRxLoraMessageMillis = 0;    //last message received
unsigned long previousRxLoraMessageMillis = 0;

unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;

void setup() {
  Serial.begin(115200);

  //lora init
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  //LoRa.setSpreadingFactor(10);   // default is 7, 6 - 12
  LoRa.enableCrc();
  //LoRa.setSignalBandwidth(500E3);   //signalBandwidth - signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.

  // display init
  display.init();
  //display.flipScreenVertically();  

  //Serial.println(" Longpress Time: " + String(btnUp.getLongClickTime()) + "ms");
  //Serial.println(" DoubleClick Time: " + String(btnUp.getDoubleClickTime()) + "ms");
  btnUp.setPressedHandler(btnPressed);
  btnDown.setPressedHandler(btnPressed);
  btnDown.setLongClickTime(500);
  btnDown.setLongClickDetectedHandler(btnDownLongClickDetected);
  btnDown.setDoubleClickTime(400);
  btnDown.setDoubleClickHandler(btnDownDoubleClick);
    
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Transmitter \n");
  display.drawString(0, 0, "Starting Transmitter");

  // admin --> scan for existing transmitter for a few seconds --> start up with his current pull state
  if (myID == 0 ) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "Searching 4s for");
      display.drawString(0, 14, "existing transmitter...");
      display.display();
      lastTxLoraMessageMillis = millis();
      while (millis() < lastTxLoraMessageMillis + 4000) {
          // packet from transmitter
          if (LoRa.parsePacket() >= sizeof(loraTxMessage) ) {
            LoRa.readBytes((uint8_t *)&loraTxMessage, sizeof(loraTxMessage));
            if (loraTxMessage.pullValue == loraTxMessage.pullValueBackup) {
                //found --> read state and exit
                currentState = loraTxMessage.currentState;
                targetPull = loraTxMessage.pullValue;
                Serial.printf("Found existing transmitter, starting up with state: %d: %d \n", currentState, targetPull);
                //exit search loop
                lastTxLoraMessageMillis = millis() - 4000;
            }
          } 
          delay(10);
       }
   }
   // reset to my transmitter id
   loraTxMessage.id = myID;
}

void loop() {

    loopStep++;
  
    // screen
    if (loopStep % 100 == 0) {
      toogleSlow = !toogleSlow;
    }
    if (loopStep % 10 == 0) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);  //10, 16, 24
      if (toogleSlow) {
          display.drawString(0, 0, loraTxMessage.id + String("-B: ") + vescBattery + "%, T: " + vescTempMotor + " C");        
      } else {
          display.drawString(0, 0, loraTxMessage.id + String("-T: ") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");        
      }
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + currentPull + String("kg)"));
      display.drawString(0, 36, String(loraRxMessage.tachometer * 10) + "m| " + String(loraRxMessage.dutyCycleNow) + "%" );
      display.display();
    }
    
    // LoRa data available?
    //==acknowledgement from receiver?
    if (LoRa.parsePacket() == sizeof(loraRxMessage) ) {
        LoRa.readBytes((uint8_t *)&loraRxMessage, sizeof(loraRxMessage));
        currentPull = loraRxMessage.pullValue;
        // vescBatteryPercentage and vescTempMotor are alternated on lora link to reduce packet size
          if (loraRxMessage.vescBatteryOrTempMotor == 1){
            vescBattery = loraRxMessage.vescBatteryOrTempMotorValue;
          } else {
            vescTempMotor = loraRxMessage.vescBatteryOrTempMotorValue;
          }
        previousRxLoraMessageMillis = lastRxLoraMessageMillis;  // remember time of previous paket
        lastRxLoraMessageMillis = millis();
        rssi = LoRa.packetRssi();
        snr = LoRa.packetSnr();
        Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraRxMessage.pullValue, rssi, snr);
        Serial.printf("tacho: %d, dutty: %d: \n", loraRxMessage.tachometer * 10, loraRxMessage.dutyCycleNow);
   }

  // if no lora message for more then 1,5s --> show error on screen + acustic
  if (millis() > lastRxLoraMessageMillis + 1500 ) {
        //TODO acustic information
        //TODO  red disply
        display.clear();
        display.display();
        // log connection error
       if (millis() > loraErrorMillis + 5000) {
            loraErrorMillis = millis();
            loraErrorCount = loraErrorCount + 1;
       }
  }
  
        // state machine
        // -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
        switch(currentState) {
            case -2:
              targetPull = hardBrake; // -> hard brake
              break;
            case -1:
              targetPull = softBrake; // -> soft brake
              break;
            case 0:
              targetPull = 0; // -> neutral, no pull / no brake
              break;
            case 1: 
              targetPull = defaultPull;   //independent of max pull
              break;
            case 2: 
              targetPull = myMaxPull * prePullScale / 100;
              break;
            case 3:
              targetPull = myMaxPull * takeOffPullScale / 100;
              break;
            case 4:
              targetPull = myMaxPull * fullPullScale / 100;
              break;
            case 5:
              targetPull = myMaxPull * strongPullScale / 100;
              break;
            default: 
              targetPull = softBrake;
              Serial.println("no valid state");
              break;
          }

        delay(10);

        // send Lora message every 400ms  --> three lost packages lead to failsafe on receiver (>1,5s)
        // send immediatly if state has changed
        if (millis() > lastTxLoraMessageMillis + 400 || stateChanged) {
            stateChanged = false;
            loraTxMessage.currentState = currentState;
            loraTxMessage.pullValue = targetPull;
            loraTxMessage.pullValueBackup = targetPull;
            if (LoRa.beginPacket()) {
                LoRa.write((uint8_t*)&loraTxMessage, sizeof(loraTxMessage));
                LoRa.endPacket();
                Serial.printf("sending value %d: \n", targetPull);
                lastTxLoraMessageMillis = millis();  
            } else {
                Serial.println("Lora send busy");
            }
        }

        btnUp.loop();
        btnDown.loop();
        delay(10);
}

void btnPressed(Button2& btn) {
    if (btn == btnUp) {
        Serial.println("btnUP pressed");
        //do not switch up to fast
        if (millis() > lastStateSwitchMillis + 1000 && currentState < 5) {
          currentState = currentState + 1;
          // skip neutral state to prevent line mess up
          if (currentState == 0 ) {
              currentState = currentState + 1;
          }
          lastStateSwitchMillis = millis();
          stateChanged = true;
        }
    } else if (btn == btnDown) {
        Serial.println("btnDown pressed");
        if (currentState > 1) {
          currentState = 1;   //default pull
          lastStateSwitchMillis = millis();
          stateChanged = true;
        } else if (currentState > -2 && currentState < 1){
          currentState = currentState - 1;
          lastStateSwitchMillis = millis();
          stateChanged = true;
        }
    }
}

void btnDownLongClickDetected(Button2& btn) {
    currentState = -1;    //brake
    lastStateSwitchMillis = millis();
    stateChanged = true;
}
void btnDownDoubleClick(Button2& btn) {
  // only get to neutral state from brake
  if (currentState <= -1) {
    currentState = 0;    // neutral
    lastStateSwitchMillis = millis();
    stateChanged = true;
  }
}

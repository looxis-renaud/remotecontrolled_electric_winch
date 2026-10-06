/* RECEIVER
 * receives target pull value from lora link
 * sends acknowlegement on lora with current parameters
 * writes target pull with PWM signal to vesc
 * reads current parameters (tachometer, battery %, motor temp) with UART from vesc, based on (https://github.com/SolidGeek/VescUart/)
 * A relay on IO12 switches the VESC cooling fan. The receiver switches it on by itself
 * while a pull state is active and off again after FAN_RUN_ON_MS without pull [WINCH-07].
 * (The emergency line cutter was removed [WINCH-05].)
 */

//vesc battery number of cells
static int numberOfCells = 16;

#include "LiPoCheck.h"    //to calculate battery % based on cell Voltage

#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <LoRa.h>

// Include the correct display library for a connection via I2C using Wire include
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"

SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL - SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23   // GPIO23 -- SX1278's RESET on TTGO LoRa32 V2.1_1.6 (was 14, which is also VESC_RX: LoRa.begin() turned the UART RX pin into an output) [WINCH-15/16]
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

// Relay for the VESC cooling fan [WINCH-07]
int relayPin = 12; //Connect Relay Red Cable to 5V, Black Cable to GND and White Cable / Signal to Pin 12
// Relay module polarity: true = IO12 HIGH switches the fan on (active-high module),
// false = IO12 LOW switches the fan on (active-low module). Check on the bench that the fan
// physically runs when the OLED shows "Fan ON" (known bug in WINCH-07).
#define RELAY_ACTIVE_HIGH  true
// Fan run-on after the last pull state (state >= 1), in ms. Lets the VESC cool down and
// avoids switching the fan on and off during step tows.
#define FAN_RUN_ON_MS  120000UL
bool relay = false;                     // fan on/off, decided by the receiver only
unsigned long lastPullStateMillis = 0;  // last time a pull state (currentState >= 1) was active
bool pullStateSeen = false;             // no run-on after power-up in brake

//Using VescUart library to read from Vesc (https://github.com/SolidGeek/VescUart/)
#include <VescUart.h>
#define VESC_RX  14    //connect to TX on Vesc
#define VESC_TX  2    //connect to RX on Vesc
VescUart vescUART;

// PWM signal to vesc
#define PWM_PIN_OUT  13 //Define Digital PIN
#define PWM_TIME_0      950.0    //PWM time in ms for 0% , PWM below will be ignored!! need XXX.0!!!
#define PWM_TIME_100    2000.0   //PWM time in ms for 100%, PWM above will be ignored!!

static int loopStep = 0;
static uint8_t activeTxId = 0;

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

//sent by transmitter
struct LoraTxMessage {
   uint8_t id : 4;              // unique id 1 - 15, id 0 is admin!
   int8_t currentState : 4;    // -2 --> -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
   int8_t pullValue;           // target pull value,  -127 - 0 --> 5 brake, 0 - 127 --> pull
   int8_t pullValueBackup;     // to avoid transmission issues, TODO remove, CRC is enough??
   // [WINCH-05/07] servo and relay fields removed: 3 bytes instead of 5. Transmitter and
   // receiver must be flashed together; packets of the old size are ignored.
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

// Packets are matched by size: both sketches must have the same sizes [WINCH-05/07]
static_assert(sizeof(LoraTxMessage) == 3, "LoraTxMessage must be 3 bytes, same as in transmitter.ino");
static_assert(sizeof(LoraRxMessage) == 4, "LoraRxMessage must be 4 bytes, same as in transmitter.ino");

int smoothStep = 0;    // used to smooth pull changes
int hardBrake = -20;  //in kg
int softBrake = -8;  //in kg
int defaultPull = 8;  //in kg
// The pull states (myMaxPull, scales) live in the transmitter only. The receiver uses the
// three values above for failsafe and autostop. Unused copies removed 2026-10-03 [WINCH-15].

int currentId = 0;
int currentState = -1;
// pull value send to VESC --> default soft brake
// defined as int to allow smooth changes without overrun
int currentPull = softBrake;     // active range -127 to 127
int8_t targetPullValue = 0;    // received from lora transmitter or rewinding winch mode

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;
bool vescUartOk = false;   // result of the last UART read, shown on the OLED [WINCH-16]

unsigned long lastTxLoraMessageMillis = 0;
unsigned long previousTxLoraMessageMillis = 0;
unsigned long lastRxLoraMessageMillis = 0;
unsigned long previousRxLoraMessageMillis = 0;
uint32_t  pwmReadTimeValue = 0;
uint32_t  pwmWriteTimeValue = 0;
unsigned long lastWritePWMMillis = 0;
unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;


void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);  //biase caused by digital write/read
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
}

void setup() {
  Serial.begin(115200);

  //Setup UART port for Vesc communication
  Serial1.begin(115200, SERIAL_8N1, VESC_RX, VESC_TX);
  vescUART.setSerialPort(&Serial1);
  //vescUART.setDebugPort(&Serial);

  // Setup Relay, fan off at power-up
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_ACTIVE_HIGH ? LOW : HIGH);

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
  display.flipScreenVertically();  

  //PWM Pins
  //pinMode(PWM_PIN_IN, INPUT);
  pinMode(PWM_PIN_OUT, OUTPUT);
  
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Receiver \n");
  display.drawString(0, 0, "Starting Receiver");
}

void loop() {

 loopStep++;
 // TODO activate rewinding winch mode here
 if (true) {
    // screen
    if (loopStep % 10 == 0) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      display.drawString(0, 0, currentId + String("-RX: (") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      if (currentState > 0){
          display.drawString(0, 11, String("P ") + currentState + ": (" + currentPull + "kg)");  
      } else {
          display.drawString(0, 11, String("B ") + currentState + ": (" + currentPull + "kg)");    
      }
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      //display.drawString(0, 36, String("Error / Uptime{min}: ") + loraErrorCount + " / " + millis()/60000);
      // display.drawString(0, 36, String("B: ") + vescBattery + "%, M: " + vescTempMotor + "C");
      if (relay == true && currentState >= 1) {
        display.drawString(0, 36, String("Fan ON"));
      } else if (relay == true) {
        // run-on: show remaining seconds
        unsigned long fanOnSinceMs = millis() - lastPullStateMillis;
        unsigned long fanRemainingS = fanOnSinceMs < FAN_RUN_ON_MS ? (FAN_RUN_ON_MS - fanOnSinceMs) / 1000 : 0;
        display.drawString(0, 36, String("Fan ON (off in ") + fanRemainingS + " s)");
      } else {
        display.drawString(0, 36, String("Fan OFF"));
      }
      // display.drawString(0, 48, String("Last TX / RX: ") + lastTxLoraMessageMillis/100 + " / " + lastRxLoraMessageMillis/100);
      // UART telemetry: raw tachometer counts and duty cycle as read from the VESC [WINCH-16]
      // e.g. "Tac 15914 D 12% ok", "ERR" = last UART read failed (values are then the last good ones)
      display.drawString(0, 48, String("Tac ") + vescUART.data.tachometer + " D "
                         + (int)abs(vescUART.data.dutyCycleNow * 100) + "% " + (vescUartOk ? "ok" : "ERR"));
      display.display();
    }
    
    // LoRa data available?
    // packet from transmitter
    if (LoRa.parsePacket() == sizeof(loraTxMessage) ) {
          LoRa.readBytes((uint8_t *)&loraTxMessage, sizeof(loraTxMessage));
          // allow only one ID to control the winch at a given time
          // after 5 seconds without a message, a new id is allowed
          if (millis() > lastTxLoraMessageMillis + 5000){
            activeTxId = loraTxMessage.id;
          }
          // The admin id 0 can always take over
          if (loraTxMessage.id == 0){
            activeTxId = loraTxMessage.id;
          }
          if (loraTxMessage.id == activeTxId && loraTxMessage.pullValue == loraTxMessage.pullValueBackup) {
              targetPullValue = loraTxMessage.pullValue;
              currentId = loraTxMessage.id;
              currentState = loraTxMessage.currentState;
              previousTxLoraMessageMillis = lastTxLoraMessageMillis;  // remember time of previous paket
              lastTxLoraMessageMillis = millis();
              rssi = LoRa.packetRssi();
              snr = LoRa.packetSnr();

              // Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraTxMessage.pullValue, rssi, snr);
              
              // send ackn after receiving a value
              delay(10);
              loraRxMessage.pullValue = currentPull;
              loraRxMessage.tachometer = abs(vescUART.data.tachometer)/1000;     // %100 --> in m, %10 --> to use only one byte for up to 2550m line lenght
              loraRxMessage.dutyCycleNow = abs(vescUART.data.dutyCycleNow * 100);     //in %
              // alternate vescBatteryPercentage and vescTempMotor value on lora link to reduce packet size
              if (loraRxMessage.vescBatteryOrTempMotor == 0){
                loraRxMessage.vescBatteryOrTempMotor = 1;
                loraRxMessage.vescBatteryOrTempMotorValue = vescBattery;
              } else {
                loraRxMessage.vescBatteryOrTempMotor = 0;
                loraRxMessage.vescBatteryOrTempMotorValue = vescTempMotor;
              }
              if (LoRa.beginPacket()) {
                  LoRa.write((uint8_t*)&loraRxMessage, sizeof(loraRxMessage));
                  LoRa.endPacket();
                  // Serial.printf("sending Ackn currentPull %d: \n", currentPull);
                  lastRxLoraMessageMillis = millis();  
              } else {
                  Serial.println("Lora send busy");
              }
              
          } else {
            Serial.println("Wrong transmitter id or backup Value:");
            Serial.println(loraTxMessage.id);
            Serial.println(loraTxMessage.pullValue);
            Serial.println(loraTxMessage.pullValueBackup);
          }
     }
  
      // if no lora message for more then 1,5s --> show error on screen + acustic
      if (millis() > lastTxLoraMessageMillis + 1500 ) {
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
      // Failsafe only when pull was active
      if (currentState >= 1) {
            // no packet for 1,5s --> failsave
            if (millis() > lastTxLoraMessageMillis + 1500 ) {
                 // A) keep default pull if connection issue during pull for up to 20 seconds
                 if (millis() < lastTxLoraMessageMillis + 20000) {
                    targetPullValue = defaultPull;   // default pull
                    currentState = 1;
                 } else {
                 // B) go to soft brake afterwards
                    targetPullValue = softBrake;     // soft brake
                    currentState = -1;
                 }
            }
      }
 } else {
      // rewinding winch mode
      // screen
      if (loopStep % 10 == 0) {
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_10);  //10, 16, 24
        display.drawString(0, 0, "rewinding winch mode");
        display.setFont(ArialMT_Plain_24);  //10, 16, 24
        display.drawString(0, 14, String(targetPullValue) + "/" + currentPull + "kg");
        display.display();
      }

      // small pull value on pull out
      // higher pull value on pull in
      if (vescUART.data.dutyCycleNow > 0.02){
        targetPullValue = 10;
      } else if (vescUART.data.dutyCycleNow < -0.02){
        targetPullValue = 17;
      } else {
        targetPullValue = -5; // no line movement --> soft brake
      }
      // ??? TODO higher pull value on fast pull out to avoid drum overshoot on line disconection ???
      
 }  // end rewind winch mode

      // auto line stop
      // (smooth to avoid line issues on main winch with rewinding winch)
      // tachometer > 2 --> avoid autostop when no tachometer values are read from uart (--> 0)
      if (vescUART.data.tachometer > 2 && vescUART.data.tachometer < 40) {
          if (targetPullValue > defaultPull){
              targetPullValue = defaultPull;
          }
          if (vescUART.data.tachometer < 20) {
              targetPullValue = softBrake;
          }
          if (vescUART.data.tachometer < 10) {
              targetPullValue = hardBrake;
          }
          Serial.println("Autostop active, target pull value:");
          Serial.println(targetPullValue);
      }
 
      // smooth changes
      // if brake --> immediately active
      if (targetPullValue < 0 ){
          currentPull = targetPullValue;
      } else {   
          // change rate e.g. max. 50 kg / second
          //reduce pull
          if (currentPull > targetPullValue) {
              smoothStep = 90 * (millis() - lastWritePWMMillis) / 1000;
              if ((currentPull - smoothStep) > targetPullValue)   //avoid overshooting
                  currentPull = currentPull - smoothStep;
              else
                  currentPull = targetPullValue;
          //increase pull
          } else if (currentPull < targetPullValue) {
              smoothStep = 65 * (millis() - lastWritePWMMillis) / 1000;
              if ((currentPull + smoothStep) < targetPullValue)   //avoid overshooting
                  currentPull = currentPull + smoothStep;
              else
                  currentPull = targetPullValue;
          }
          //Serial.println(currentPull);
          //avoid overrun
          if (currentPull < -127)
            currentPull = -127;
          if (currentPull > 127)
            currentPull = 127;
      }
      
      delay(10);
      //calculate PWM time for VESC
      // write PWM signal to VESC
      pwmWriteTimeValue = (currentPull + 127) * (PWM_TIME_100 - PWM_TIME_0) / 254 + PWM_TIME_0;     
      pulseOut(PWM_PIN_OUT, pwmWriteTimeValue);
      lastWritePWMMillis = millis();
      delay(10);    //RC PWM usually has a signal every 20ms (50 Hz)

  // Relay for the VESC cooling fan [WINCH-07]
  // ON while a pull state is active (including failsafe default pull),
  // OFF after FAN_RUN_ON_MS without any pull state.
     if (currentState >= 1) {
       lastPullStateMillis = millis();
       pullStateSeen = true;
     }
     relay = pullStateSeen && (millis() - lastPullStateMillis < FAN_RUN_ON_MS);
     if (relay) {
       digitalWrite(relayPin, RELAY_ACTIVE_HIGH ? HIGH : LOW); // fan on
     } else {
       digitalWrite(relayPin, RELAY_ACTIVE_HIGH ? LOW : HIGH); // fan off
     }

      //read actual Vesc values from uart
      if (loopStep % 20 == 0) {
        vescUartOk = vescUART.getVescValues();
        if (vescUartOk) {
            vescBattery = CapCheckPerc(vescUART.data.inpVoltage, numberOfCells);    // vesc battery in %
            vescTempMotor = vescUART.data.tempMotor;                                // motor temp in C
            // UART diagnosis on the USB serial monitor (115200 baud) [WINCH-16]
            Serial.printf("VESC ok: %.1f V, tacho %ld, duty %.2f, motor %.0f C\n",
                          vescUART.data.inpVoltage, (long)vescUART.data.tachometer,
                          vescUART.data.dutyCycleNow, vescUART.data.tempMotor);
            //SerialPrint(measuredVescVal, &DEBUGSERIAL);
            /*
            Serial.println(vescUART.data.tachometer);
            Serial.println(vescUART.data.inpVoltage);
            Serial.println(vescUART.data.dutyCycleNow);            
            Serial.println(vescUART.data.tempMotor);
            Serial.println(vescUART.data.tempMosfet);
            vescUART.printVescValues();
            */
          }
        else
          {
            //TODO send notification to lora
            //measuredVescVal.tachometer = 0;
            Serial.println("Failed to get data from VESC!");   // UART diagnosis [WINCH-16]
          }
      }
}

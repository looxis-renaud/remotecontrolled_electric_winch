/* RECEIVER
 * receives target pull value from lora link
 * sends acknowlegement on lora with current parameters
 * writes target pull with PWM signal to vesc
 * reads current parameters (tachometer, battery %, motor temp) with UART from vesc, based on (https://github.com/SolidGeek/VescUart/)
 * Update: Will add support for triggering a Servo and Relay
 */

//vesc battery number of cells
static int numberOfCells = 16;
static int myMaxPull = 85;  // 0 - 127 [kg], must be scaled with VESC ppm settings

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

// Servo and Relay Library
#include <Servo.h> // https://github.com/arduino-libraries/Servo
#include "Relay.h" // https://github.com/rafaelnsantos/Relay

int8_t deployServo = 0;   // variable for Servo - used to trigger an emergency line cutter. Needs to be implemented
bool relayOn = false;     // variable for Relay - used to turn on fan and warning light. is first off, and will be turned on by switching on the remote


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
   int8_t deployServo;         // is supposed to send Servo position for emergency line cutter
   bool relayOn : true;       // is supposed to turn relay on and off. Fan and warning light will be connected to relay
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

int smoothStep = 0;    // used to smooth pull changes
int hardBrake = -20;  //in kg
int softBrake = -8;  //in kg
int defaultPull = 8;  //in kg
int prePullScale = 20;      //in % of myMaxPull
int takeOffPullScale = 50;  //in % of myMaxPull
int fullPullScale = 80;     //in % of myMaxPull
int strongPullScale = 100;  //in % of myMaxPull

int currentId = 0;
int currentState = -1;
// pull value send to VESC --> default soft brake
// defined as int to allow smooth changes without overrun
int currentPull = softBrake;     // active range -127 to 127
int8_t targetPullValue = 0;    // received from lora transmitter or rewinding winch mode

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;

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
      display.drawString(0, 36, String("B: ") + vescBattery + "%, M: " + vescTempMotor + "C");
      display.drawString(0, 48, String("Last TX / RX: ") + lastTxLoraMessageMillis/100 + " / " + lastRxLoraMessageMillis/100);
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
              Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraTxMessage.pullValue, rssi, snr);
              
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
                  Serial.printf("sending Ackn currentPull %d: \n", currentPull);
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
                 // A) keep default pull if connection issue during pull for up to 10 seconds
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


     // ToDo: Emergeny Line Cutter
     // if (deployServo == 90) {
         //trigger Servo
     // }

     // ToDo: Relay on and off logic
     //   if (relayOn == true) {
           //turn on Relay
     //   else
     //      //turn off Relay
     //   }
   

      
      //read actual Vesc values from uart
      if (loopStep % 20 == 0) {
        if (vescUART.getVescValues()) {
            vescBattery = CapCheckPerc(vescUART.data.inpVoltage, numberOfCells);    // vesc battery in %
            vescTempMotor = vescUART.data.tempMotor;                                // motor temp in C            
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
            Serial.println("Failed to get data from VESC!");
          }
      }
}

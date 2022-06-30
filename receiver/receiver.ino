/*
 * receiver
 * receives target pull value from lora link
 * sends acknowlegement on lora with current parameters
 * writes target pull with PWM signal to vesc
 * reads current parameters (tachometer, battery %, motor temp) with UART from vesc, based on (https://github.com/SolidGeek/VescUart/)
 * 
 */

//vesc battery number of cells
static int numberOfCells = 16;

#include "LiPoCheck.h"    //to calculate battery % based on cell Voltage

#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h"
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  868E6

SSD1306 display(0x3c, 21, 22);
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


//Using VescUart librarie to read from Vesc (https://github.com/SolidGeek/VescUart/)
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

#include "common.h"
struct LoraTxMessage loraTxMessage;
struct LoraRxMessage loraRxMessage;

int currentId = 0;
int currentState = -1;
int currentPull = 3;    // pull value send to VESC
uint8_t loraPullValue = 0;    // received from lora transmitter
int smoothStep = 0;    // used to smooth pull changes
int defaultPullScale = 11;  //in %
int prePullScale = 20;      //in %
int takeOffPullScale = 50;  //in %
int fullPullScale = 80;     //in %
int strongPullScale = 100;  //in %

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
  
  //OLED display
  pinMode(16,OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

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
  
    // screen
    if (loopStep % 10 == 0) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      display.drawString(0, 0, currentId + String("-RX: (") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      if (currentState > 0){
          display.drawString(0, 11, String("P ") + currentState + ": (" + currentPull + ")");  
      } else {
          display.drawString(0, 11, String("B ") + currentState + ": (" + currentPull + ")");    
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
      // The admin id 0 can allways take over
      if (loraTxMessage.id == 0){
        activeTxId = loraTxMessage.id;
      }
      if (loraTxMessage.id == activeTxId && loraTxMessage.pullValue == loraTxMessage.pullValueBackup) {
          loraPullValue = loraTxMessage.pullValue;
          currentId = loraTxMessage.id;
          currentState = loraTxMessage.currentState - 2;
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
        //TODO  red disply
        display.clear();
        display.display();
        // log connection error
       if (millis() > loraErrorMillis + 5000) {
            loraErrorMillis = millis();
            loraErrorCount = loraErrorCount + 1;
       }
  }


  // Failsafe only when pull was active
  if (loraPullValue >= (255 * defaultPullScale / 100)) {
        // no packet for 1,5s --> failsave
        if (millis() > lastTxLoraMessageMillis + 1500 ) {
             // A) keep default pull if connection issue during pull for up to 10 seconds
             if (millis() < lastTxLoraMessageMillis + 20000) {
                loraPullValue = 255 * defaultPullScale / 100;   //default pull
             } else {
             // B) go to soft brake afterwards
                loraPullValue = 4;
             }
        }
  }
      
      //calculate PWM time for VESC
      if (loraPullValue > 0 ){
          // smooth changes --> change rate e.g. max. 200 / second
          //reduce pull
          if (currentPull > loraPullValue) {
              smoothStep = 200 * (millis() - lastWritePWMMillis) / 1000;
              if ((currentPull - smoothStep) > loraPullValue)   //avoid overshooting
                  currentPull = currentPull - smoothStep;
              else
                  currentPull = loraPullValue;
          //increase pull
          } else if (currentPull < loraPullValue) {
              smoothStep = 120 * (millis() - lastWritePWMMillis) / 1000;
              if ((currentPull + smoothStep) < loraPullValue)   //avoid overshooting
                  currentPull = currentPull + smoothStep;
              else
                  currentPull = loraPullValue;
          }
          //avoid overrun
          if (currentPull < 0)
            currentPull = 0;
          if (currentPull > 255)
            currentPull = 255;
      } else {   // if 0 --> immediatly stop
        currentPull = 0;
      }
      
      delay(10);
      //TODO add auto line stopp here

      // write PWM signal to VESC
      pwmWriteTimeValue = currentPull * (PWM_TIME_100 - PWM_TIME_0) / 255 + PWM_TIME_0;     
      pulseOut(PWM_PIN_OUT, pwmWriteTimeValue);
      lastWritePWMMillis = millis();
      delay(10);    //RC PWM usually has a signal every 20ms (50 Hz)

      
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

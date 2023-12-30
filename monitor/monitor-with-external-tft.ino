/*
 * monitor
 * shows current transmitter and receiver status
 * does only listen to lora messages
 * updated to support an external TFT Display
 */

#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <LoRa.h>

// Include the correct display library for a connection via I2C using Wire include
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

// Initialize the OLED display using Wire library
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA (Pin 21), SCL (Pin 22) - SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23 //14  // GPIO14 -- SX1278's RESET // switching to the board's natural pin 23 (according to PINOUT scheme)
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  868E6  //frequency in Hz (433E6, 868E6, 915E6) 

int rssi = 0;
float snr = 0;

#include <rom/rtc.h>

#include "Arduino.h"

// battery measurement
//#define CONV_FACTOR 1.7
//#define READS 20
Pangodream_18650_CL BL(35); // pin 34 old / 35 new v2.1 hw


static int loopStep = 0;
bool toogleSlow = true;
int8_t targetPull = 0;   // pull value range from -127 to 127
int currentPull = 0;          // current active pull on vesc
bool stateChanged = false;
int currentState = 0;   // -1 = stopped/brake, 0 = no pull/no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
unsigned long lastStateSwitchMillis = 0;

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;

#include "common.h"
struct LoraTxMessage loraTxMessage;
struct LoraRxMessage loraRxMessage;

unsigned long lastTxLoraMessageMillis = 0;    //last message send
unsigned long lastRxLoraMessageMillis = 0;    //last message received
unsigned long previousTxLoraMessageMillis = 0;
unsigned long previousRxLoraMessageMillis = 0;

unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;

/*
  Adding TFT_eSPI library (Created by Bodmer 31/12/16) functionality
  This allows to add a larger TFT Display powered by SPI

  You can use print class and drawString() functions
  to plot text to the screen.


  --------------------------- PIN SETUP for TFT Display -------------------
 //#define TFT_MISO 19 
 #define TFT_MOSI 12
 #define TFT_SCLK 13
 #define TFT_CS   15  // Chip select control pin
 #define TFT_DC   2  // Data Command control pin
 //#define TFT_RST  4  // Reset pin (could connect to RST pin)
 #define TFT_RST  -1  // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST
  -------------------------------------------------------------------------

  ##################################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE TFT_eSPI LIBRARY ######
  ######       TO SELECT YOUR DISPLAY TYPE AND ENABLE FONTS                   ######
  ##################################################################################
*/

// #include "Free_Fonts.h" // let's try without free fonts
#include <TFT_eSPI.h>

// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();


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

  //start OLED Display
  display.init();
  display.flipScreenVertically();  
  
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Monitor \n");
  display.drawString(0, 0, "Starting Monitor");

  // external TFT Display initialisation
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);            // Clear screen
 // tft.drawString("Starting Monitor", 0, 0, GFXFF);
}


void loop() {
    loopStep++;
  
    // screen
    if (loopStep % 100 == 0) {
      toogleSlow = !toogleSlow;
    }
    if (loopStep % 100 == 0) {
      tft.fillScreen(TFT_BLACK); //clears the screen, but let's it flicker like crazy, hence a higher loopStep Value
      }
    if (loopStep % 10 == 0) {

      tft.setCursor(0, 0, 2);                   // Set "cursor" at top left corner of display (0,0) and select font 2
 // (cursor will move to next line automatically during printing with 'tft.println'
  //  or stay on the line is there is room for the text with tft.print)
      tft.setTextColor(TFT_WHITE, TFT_BLACK); //white Text with black background
      // OLED Display: 
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);  //10, 16, 24
      if (toogleSlow) {
          display.drawString(0, 0, String("B: ") + vescBattery + "%, T: " + vescTempMotor + " C");        
      } else {
          display.drawString(0, 0, String("T:") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");        
      }
      tft.setTextSize(2);
      tft.println(String("State ") + String(currentState) );
      tft.setTextSize(3);
      tft.println(String(String("Pull ") + targetPull + "/" + currentPull + String("kg") ));
      tft.println(String("Line ") + String(loraRxMessage.tachometer) + " m / " + String(loraRxMessage.dutyCycleNow) + "%" );
      tft.setTextSize(1);
      tft.println(String("VescBatt: ") + vescBattery + "% | VescTemp: " + vescTempMotor + " C" );
      tft.println(String("Antenna: ") + rssi + String("dBm, ") + snr );
      tft.println(String("Remote ID: ") + String(loraTxMessage.id) );
//      tft.fillScreen(TFT_BLACK); clears the screen, but let's it flicker like crazy
      display.setFont(ArialMT_Plain_16);  //10, 16, 24
      display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + currentPull + String("kg)"));
      display.drawString(0, 36, "ID: " + String(loraTxMessage.id) + " - " + String(loraRxMessage.tachometer) + " m | " + String(loraRxMessage.dutyCycleNow) + "%");
      display.display();
    }
    
    // LoRa data available?
    int loraPacketSize = 0;
    loraPacketSize = LoRa.parsePacket();
    // == packet from transmitter?
    if (loraPacketSize == sizeof(loraTxMessage) ) {
      LoRa.readBytes((uint8_t *)&loraTxMessage, sizeof(loraTxMessage));
      if ( loraTxMessage.pullValue == loraTxMessage.pullValueBackup) {
          targetPull = loraTxMessage.pullValue;
          currentState = loraTxMessage.currentState;
          previousTxLoraMessageMillis = lastTxLoraMessageMillis;  // remember time of previous paket
          lastTxLoraMessageMillis = millis();
          rssi = LoRa.packetRssi();
          snr = LoRa.packetSnr();
          Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraTxMessage.pullValue, rssi, snr);
      }
   }
    //==acknowledgement from receiver?
    if (loraPacketSize == sizeof(loraRxMessage) ) {
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
        Serial.printf("tacho: %d, dutty: %d: \n", loraRxMessage.tachometer, loraRxMessage.dutyCycleNow);
   }

  // if no lora message for more then 1,5s --> show error on screen + acustic
  if (millis() > lastRxLoraMessageMillis + 1500 ) {
        //TODO acustic information
        if (loopStep % 100 == 0) {
          tft.fillScreen(TFT_RED); //clears the screen, but let's it flicker like crazy, hence a higher loopStep Value
         }
        display.clear();
        display.display();
        // log connection error
       if (millis() > loraErrorMillis + 5000) {
            loraErrorMillis = millis();
            loraErrorCount = loraErrorCount + 1;
       }
  }
        delay(10);
}

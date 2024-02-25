/*
 * electric paraglider winch remote control monitor 
 * use it on an Lilygo T-Display S3 to display winch values on your paraglider Cockpit.
 * This monitor connects with the TTGO Lora ESP32 Paxounter Remote Transmitter via ESP-NOW
 * to display useful data in your eye-sight (rather than having to turn your head
 * to view the onboard OLED display of your remote)
 * Code Basis of ESP-Now communication by Rui Santos
 * Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files.
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// TFT Display
#include <TFT_eSPI.h>
// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();

//unsigned long drawTime = 0;

//#include "Free_Fonts.h"


// Structure to receive data
// Must match the sender structure
struct EspNowTxMessage {
  int8_t pullValue;
  int currentPull;
  int8_t currentState;
  bool servo;
  bool relay;
  uint8_t tachometer;
  uint8_t dutyCycleNow;
} ;

// Create a struct message called EspNowTXMessage
struct EspNowTxMessage EspNowTxMessage;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&EspNowTxMessage, incomingData, sizeof(EspNowTxMessage));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Target Pull: ");
  Serial.println(EspNowTxMessage.pullValue);
  Serial.print("Current Pull: ");
  Serial.println(EspNowTxMessage.currentPull);
  Serial.print("State: ");
  Serial.println(EspNowTxMessage.currentState);
  Serial.print("Servo: ");
  Serial.println(EspNowTxMessage.servo);
  Serial.print("Relay: ");
  Serial.println(EspNowTxMessage.relay);
  Serial.print("Line: ");
  Serial.println(EspNowTxMessage.tachometer);
  Serial.print("Speed%: ");
  Serial.println(EspNowTxMessage.dutyCycleNow);
  Serial.println();

  draw();

}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize TFT Display
  pinMode(15,OUTPUT);
  digitalWrite(15,1);

  tft.init();
  tft.setRotation(1);

  ledcSetup(0, 10000, 8);
  ledcAttachPin(38, 0);
  ledcWrite(0, 110);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  //if (esp_now_init() != ESP_OK) {
  //  Serial.println("Error initializing ESP-NOW");
  //  return;
  //}
  // Alternatively init ESP-NOW without confirmation to the serial port:
 esp_now_init();
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void draw()
  {
    tft.fillScreen(TFT_BLACK); // Clear the screen
    tft.setCursor(0, 0, 2);
   // Set the font colour to be white with a black background, set text size multiplier to 1
    tft.setTextColor(TFT_WHITE,TFT_BLACK);  tft.setTextSize(3);
   // tft.setFreeFont(FF18);
    tft.print("P "); tft.print(EspNowTxMessage.currentState); tft.print(": "); tft.print(EspNowTxMessage.pullValue); tft.print("/"); tft.print(EspNowTxMessage.currentPull); tft.println(" kg");
  }

 
void loop() {

}

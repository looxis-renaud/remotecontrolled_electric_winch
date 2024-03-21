/*
 * Electric Paraglider Winch Monitor 
 * use it on an Lilygo T-Display S3 to display winch values on your paraglider Cockpit
 * and to change maxPull Value pre-flight.
 * This monitor connects with the TTGO Lora ESP32 Paxounter Remote Transmitter via ESP-NOW
 * to display useful data in your eye-sight (rather than having to turn your head
 * to view the onboard OLED display of your remote)
 * UPDATE: Added functionality to not only use the T-Display as a Monitor,
 * but also to accept button presses that are sent to the Remote Transmitter
 * for additional control (Servo, Relay and maximum Pull Value)
 * Code Basis of ESP-Now communication by Rui Santos
 * Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files.
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
*/

#include "Arduino.h" 
#include "pin_config.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Button2.h>

// Mac Address of ESP-NOW Receiver Board / Remote Control Transmitter
uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x9D, 0x61, 0x18}; 

// TFT Display
#include <TFT_eSPI.h>
// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();

// Buttons for Relay, Servo and MaxPull control
#define BUTTON_A  0 // up
#define BUTTON_B  14 // down
#define BUTTON_C  12 // settings/maxPull control

Button2 btnA = Button2(BUTTON_A);
Button2 btnB = Button2(BUTTON_B);
Button2 btnC = Button2(BUTTON_C);

 // Variables to store incoming/outgoing data
  int8_t pullValue;
  int currentPull;
  int setMaxPull = 85;
  int8_t currentState;
  bool servo;
  bool relay;
  uint8_t tachometer;
  uint8_t dutyCycleNow;

//Variable for Serial.Print Success Message
String success;

// Structure to send data
// Must match the Transmitters structure
struct EspNowButtonMessage {
  bool servo;
  bool relay;
  int setMaxPull = 85;  // Initial value for maxPull / Add functionality to change maxPull Value on Transmitter
} ;

// Create a struct message called EspNowButtonMessage
struct EspNowButtonMessage EspNowButtonMessage; 

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

//Variable that holds Information about the peer
esp_now_peer_info_t peerInfo;

// Variables for Settings Control
const int potPin = 16;     // Pin connected to the potentiometer & 3,3V, reads a value between 0 and 4095
int potValue = 0;          // Variable to store the potentiometer value
int mappedValue = 0;       // Variable to store the mapped value
bool settingsIsActive = false;     // Flag to track the state of the potentiometer

int xpos = 10;

// callback function that will be executed when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
// Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
// if (status ==0) {
//   success = "Delivery Success :)";
// }
//   else {
//     success = "Delivery Fail";
//   }
// }

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (!settingsIsActive) {
    // Only update EspNowTxMessage if settings are not active
  memcpy(&EspNowTxMessage, incomingData, sizeof(EspNowTxMessage));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  draw();
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Serial.println("Hello T-Display-S3");

  // Initialize TFT Display
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  tft.begin();
  tft.setRotation(3);
  tft.setSwapBytes(true);

  ledcSetup(0, 10000, 8);
  ledcAttachPin(38, 0);
  ledcWrite(0, 110);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  esp_now_init();
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Register for the OnDataSent callback function
  //esp_now_register_send_cb(OnDataSent);

  // Register Peer (Remote Control) to receive Data
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer for ESP-NOW       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Setup Buttons
  btnA.setPressedHandler(btnAPressed);
  // btnA.setLongClickTime(500);
  // btnA.setLongClickDetectedHandler(btnALongClickDetected);

  btnB.setPressedHandler(btnBPressed);
  btnB.setDoubleClickTime(400);
  btnB.setDoubleClickHandler(btnBDoubleClick);

  btnC.setPressedHandler(btnCPressed);
}

void draw() {
  tft.fillScreen(TFT_BLACK); // Clear the screen
  if (settingsIsActive) {
  // Display setMaxPull value if settings are active
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Set MaxPull Value: " + String(setMaxPull) + "kg", 0, 0, 4);
  } else {
  // Display received ESP-Now values if settings are not active
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("TargetPull: " + String(EspNowTxMessage.pullValue) + "kg", 0, 0, 4);
  tft.drawString(String(EspNowTxMessage.currentPull), 200, 30, 8); // Display current pull
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Line: " + String(EspNowTxMessage.tachometer) + "m", 0, 40, 4); // Display deployed line length in meters
  tft.drawString("Speed: " + String(EspNowTxMessage.dutyCycleNow) + "%", 0 ,75, 4);  // Display %of max Speed before VESC goes bust
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  int xpos = 0;
  xpos += tft.drawString("Fan: " + String(EspNowTxMessage.relay ? "ON" : "OFF") + " | ", 0, 120, 2); // Display servo state
  tft.drawString("LineCutter: " + String(EspNowTxMessage.servo ? "EMERGENCY" : "Ready!"), xpos, 120, 2); // Display servo state
  }
}

 
void loop() {

    // Update maxPull value only if potentiometer is active
  if (settingsIsActive) {
    // Read the value from the potentiometer
    potValue = analogRead(potPin);
    
    // Map the potentiometer value to the range of possible maxPull values
    mappedValue = map(potValue, 0, 4095, 0, 12);
    
    // Select the corresponding maxPull value from the list
    switch (mappedValue) {
      case 0:
        setMaxPull = 60;
        break;
      case 1:
        setMaxPull = 65;
        break;
      case 2:
        setMaxPull = 70;
        break;
      case 3:
        setMaxPull = 75;
        break;
      case 4:
        setMaxPull = 80;
        break;
      case 5:
        setMaxPull = 85;
        break;
      case 6:
        setMaxPull = 90;
        break;
      case 7:
        setMaxPull = 95;
        break;
      case 8:
        setMaxPull = 100;
        break;
      case 9:
        setMaxPull = 105;
        break;
      case 10:
        setMaxPull = 110;
        break;
      case 11:
        setMaxPull = 115;
        break;
      case 12:
        setMaxPull = 120;
        break;
    }
  }


  btnA.loop();
  btnB.loop();
  btnC.loop();
  // Serial.println(setMaxPull);
  delay(10);
}

//handle Button Presses
void btnAPressed(Button2& btn) {
  Serial.println("Button A Pressed");
  if (relay == true) {
  // Serial.println("Fan and Light turned off");
  relay = false; // turns relay off, which will deactivate Vesc Cooling Fan and Warning Light
  // lastStateSwitchMillis = millis();
  // stateChanged = true;
  } else {
  // Serial.println("Fan and Light turned on");
  relay = true; // turns relay on, cooling and warning runs again
  // lastStateSwitchMillis = millis();
  // stateChanged = true;
  }
  sendEspNowMessage(); // Send ESP-Now message when button A is pressed
 }

void btnBPressed(Button2& btn) {
// Serial.println("Button B Pressed");
  servo = true; // use only in emergency, this will trigger a line cutter, yet to be built -> Bernd, deine Aufgabe!
  sendEspNowMessage(); // Send ESP-Now message when button B is pressed
    // currentState = -2;    //hard brake, when line is being cut, of course!
    // lastStateSwitchMillis = millis();
    // stateChanged = true;
  }

void btnBDoubleClick(Button2& btn) {
//  Serial.println("Double Click on Button B");
  servo = false; // returns servo to neutral
  sendEspNowMessage(); // Send ESP-Now message when button B is double clicked
  // lastStateSwitchMillis = millis();
  // stateChanged = true;
  }

void btnCPressed(Button2& btn) {
  settingsIsActive = !settingsIsActive; // Flip the value of settingsIsActive
  if (settingsIsActive) {
    draw(); // Execute draw() when settingsIsActive is true
    // Serial.println("Button C Pressed / Settings Active\n");
  } else {
    sendEspNowMessage(); // send the values to the transmitter
    //Serial.println("Inactive\n");
  }
}

void sendEspNowMessage () {
    // prepare ESP-NOW Message 
    EspNowButtonMessage.servo = servo;
    EspNowButtonMessage.relay = relay;
    EspNowButtonMessage.setMaxPull = setMaxPull;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &EspNowButtonMessage, sizeof(EspNowButtonMessage));
    // Check whether sending the ESP-NOW Message was successful 
    //    if (result == ESP_OK) {
    //      Serial.println("Sent with success");
    //      } else {
    //      Serial.println("Error sending the data");
        }
}
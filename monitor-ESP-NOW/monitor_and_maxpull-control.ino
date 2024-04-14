/*
 * Electric Paraglider winch remote control monitor 
 * use it on an Lilygo T-Display S3 to display winch values on your paraglider Cockpit.
 * This monitor connects with the TTGO Lora ESP32 Paxounter Remote Transmitter via ESP-NOW
 * to display useful data in your eye-sight (rather than having to turn your head
 * to view the onboard OLED display of your remote)
 * UPDATE: Added functionality to not only use the T-Display as a Monitor,
 * but also to accept button presses that are sent to the Remote Transmitter
 * for additional control (Servo, Relay and maximum Pull Value).
 * Added a Rotary Encoder to change Max Pull Settings and send to Transmitter.
 * Code Basis of ESP-Now communication by Rui Santos
 * Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files.
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * Use of Sprites and Progress bar copied from https://github.com/VolosR/PbarrTut
*/

#include "Arduino.h" 
#include <esp_now.h>
#include <WiFi.h>
#include <Button2.h>
#include "AiEsp32RotaryEncoder.h" // Library for Rotary Encoder / https://github.com/igorantolic/ai-esp32-rotary-encoder

// Mac Address of ESP-NOW Receiver Board / Remote Control Transmitter
uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xA7, 0x73, 0xFC}; 

// TFT Display - make sure to correctly define the config file in TFT_eSPI Library Folder!!
#include <TFT_eSPI.h>
// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();
// Use sprites to avoid flickering of display
TFT_eSprite sprite = TFT_eSprite(&tft);
// Colors for sprites
unsigned short gray=0x6B6D; //grey color
int maxpull_segments=0; //segments for progress bar of "setMaxPull"
int currentpull_segments=0; //segments for progress bar of "currentPull"


#define PIN_POWER_ON 15

// Buttons & Rotary Encoder for Relay, Servo and MaxPull control
#define BUTTON_A  0 // integrated upper left button - for relay
#define BUTTON_B  11 // extra button - for servo
#define BUTTON_C  14 // integrated lower left button settings/maxPull control - 
#define ROTARY_ENCODER_BUTTON_PIN 12 // Button Pin "SW" on Rotary Encoder
#define ROTARY_ENCODER_A_PIN 2 // CLK Pin on Rotary Encoder
#define ROTARY_ENCODER_B_PIN 3 // DT Pin on Torary Encoder

#define ROTARY_ENCODER_STEPS 4 //depending on your encoder - try 1,2 or 4 to get expected behaviour

Button2 btnA = Button2(BUTTON_A);
Button2 btnB = Button2(BUTTON_B);
Button2 btnC = Button2(BUTTON_C);

// Set up Rotary Encoder
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);


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
bool settingsIsActive = false;     // Flag to track the state of settings mode
int rotaryDialValue=0; // variable for rotaryEncoder


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&EspNowTxMessage, incomingData, sizeof(EspNowTxMessage));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
}

// function to read Encoder
void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Serial.println("Hello T-Display-S3");

  // Initialize TFT Display
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
  sprite.createSprite(320,170);

  ledcSetup(0, 10000, 8);
  ledcAttachPin(38, 0);
  ledcWrite(0, 160);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  esp_now_init();
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

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
  btnB.setPressedHandler(btnBPressed);
  btnB.setDoubleClickTime(400);
  btnB.setDoubleClickHandler(btnBDoubleClick);
  btnC.setPressedHandler(btnCPressed);

  // Set up Rotary Encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 18, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setAcceleration(10);
}

 void draw()
  {
   sprite.setTextSize(1);
   sprite.fillSprite(TFT_BLACK);
    if (settingsIsActive) {
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("Set Maximum Pull:",10,10, 4);
      int xpos = 0;
      xpos += sprite.drawString(String(setMaxPull),10,50,7); // ,x,y,z -> x = x position, y = y position, z = size of text
      for(int i=0;i<22;i++)
        if(i<maxpull_segments)
         {sprite.drawWedgeLine(4+(i*14), 160, 12+(i*14), 140, 4, 4, TFT_WHITE,TFT_BLACK);}
          else
        {sprite.drawWedgeLine(4+(i*14), 160, 12+(i*14), 140, 4, 4, gray,TFT_BLACK);}
      sprite.drawString("kg", xpos + 10,80, 4);
    } else {
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("TargetPull: ", 0, 0, 2);
      sprite.drawString(String(EspNowTxMessage.pullValue) + "kg", 0, 20, 4);
      for(int i=0;i<22;i++)
        if(i<currentpull_segments)
         {sprite.drawWedgeLine(4+(i*14), 80, 12+(i*14), 60, 4, 4, TFT_WHITE,TFT_BLACK);}
          else
        {sprite.drawWedgeLine(4+(i*14), 80, 12+(i*14), 60, 4, 4, gray,TFT_BLACK);}
      int xpos = 140;
      xpos += sprite.drawString(String(EspNowTxMessage.currentPull), xpos, 0, 7); // Display current pull
      sprite.drawString("kg", xpos + 5, 0, 2);
      sprite.setTextColor(TFT_YELLOW);
      xpos = 0;
      xpos += sprite.drawString("Line: " + String(EspNowTxMessage.tachometer) + "m |", 0, 100, 4); // Display deployed line length in meters
      sprite.drawString("Speed: " + String(EspNowTxMessage.dutyCycleNow) + "%", xpos + 5 ,100, 4);  // Display %of max Speed before VESC goes bust
      sprite.setTextColor(TFT_MAGENTA);
      xpos = 0;
      xpos += sprite.drawString("Fan: " + String(EspNowTxMessage.relay ? "ON" : "OFF") + " | ", 0, 140, 2); // Display servo state
      sprite.drawString("LineCutter: " + String(EspNowTxMessage.servo ? "EMERGENCY" : "Ready!"), xpos, 140, 2); // Display servo state
    }
  sprite.pushSprite(0,0);
 }

 
void loop() {

    // Update maxPull value only if settings is active
  if (settingsIsActive) {
    // Read value from Rotary Encoder and assign it to variable rotarydialValue
    rotaryDialValue = rotaryEncoder.readEncoder();
    
    // Select the corresponding maxPull value from the list
    switch (rotaryDialValue) {
      case 0:
        setMaxPull = 30;
        break;
      case 1:
        setMaxPull = 35;
        break;
      case 2:
        setMaxPull = 40;
        break;
      case 3:
        setMaxPull = 45;
        break;
      case 4:
        setMaxPull = 50;
        break;
      case 5:
        setMaxPull = 55;
        break;
      case 6:
        setMaxPull = 60;
        break;
      case 7:
        setMaxPull = 65;
        break;
      case 8:
        setMaxPull = 70;
        break;
      case 9:
        setMaxPull = 75;
        break;
      case 10:
        setMaxPull = 80;
        break;
      case 11:
        setMaxPull = 85;
        break;
      case 12:
        setMaxPull = 90;
        break;
      case 13:
        setMaxPull = 95;
        break;
      case 14:
        setMaxPull = 100;
        break;
      case 15:
        setMaxPull = 105;
        break;
      case 16:
        setMaxPull = 110;
        break;
      case 17:
        setMaxPull = 115;
        break;
      case 18:
        setMaxPull = 120;
        break;
    }
  }

  btnA.loop();
  btnB.loop();
  btnC.loop();

if (rotaryEncoder.isEncoderButtonClicked()) {
    settingsIsActive = !settingsIsActive; // Flip the value of settingsIsActive;
    if (settingsIsActive) {
    // Serial.println("Rotary Encoder Button pressed / Settings Active\n");
  } else {
    sendEspNowMessage(); // send the values to the transmitter
    // Serial.println("Inactived via Rotary Encoder\n");
  }
}

  maxpull_segments = map(setMaxPull,30,120,0,22); // map the max Pull values to the 12 segments
  currentpull_segments = map(EspNowTxMessage.currentPull,0,setMaxPull,0,22); // map the max Pull values to the 12 segments

  draw();

}

//handle Button Presses
void btnAPressed(Button2& btn) {
  Serial.println("Button A Pressed");
  if (relay == true) {
  // Serial.println("Fan and Light turned off");
  relay = false; // turns relay off, which will deactivate Vesc Cooling Fan and Warning Light
  } else {
  // Serial.println("Fan and Light turned on");
  relay = true; // turns relay on, cooling and warning runs again
  }
  sendEspNowMessage(); // Send ESP-Now message when button A is pressed
 }

void btnBPressed(Button2& btn) {
 Serial.println("Button B Pressed");
  servo = true; // use only in emergency, this will trigger a line cutter
  sendEspNowMessage(); // Send ESP-Now message when button B is pressed
  }

void btnBDoubleClick(Button2& btn) {
  Serial.println("Double Click on Button B");
  servo = false; // returns servo to neutral
  sendEspNowMessage(); // Send ESP-Now message when button B is double clicked
  }

void btnCPressed(Button2& btn) {
  settingsIsActive = !settingsIsActive; // Flip the value of settingsIsActive
  if (settingsIsActive) {
    // Serial.println("Button C Pressed / Settings Active\n");
  } else {
    sendEspNowMessage(); // send the values to the transmitter
    // Serial.println("Inactive\n");
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
        // if (result == ESP_OK) {
        //   Serial.println("Sent with success");
        //   } else {
        //   Serial.println("Error sending the data");
        // }
}

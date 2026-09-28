// this code displays the mac address of the board
// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

#include <TFT_eSPI.h>
#include "WiFi.h"
TFT_eSPI tft = TFT_eSPI();

void setup(){
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  //set brightness
  ledcSetup(0, 10000, 8); ledcAttachPin(38, 0); ledcWrite(0, 100);
  
  //print mac adress
  WiFi.mode(WIFI_MODE_STA);
  tft.drawString("my MAC is:",10,10,4);
  tft.drawString(String(WiFi.macAddress()),10,50,4);

  Serial.begin(115200);
  Serial.println(WiFi.macAddress()); // print MAC Address via Serial Monitor
}
 
void loop(){

}



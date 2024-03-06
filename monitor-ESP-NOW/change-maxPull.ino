/*
* This code checks whether a button has been pressed (which puts a potentiometer
* in "active" status) and then reads the analog value from a potentiometer connected to pin A0,
* maps it to the range of possible maxPull values, and selects the corresponding value from the list.
* Then, it outputs the selected maxPull value via serial communication.
* ToDo: Test this, and if it works, implement this into the ESP-Now Monitor,
* to finally be able to change the maxPULL value from 60-120kg in 5kg steps on the Transmitter
* the button should serve as a selector to indicate whether the monitor is in "fly" mode
* or in "settings" mode
*/

const int potPin = A0;     // Pin connected to the potentiometer
const int buttonPin = 2;   // Pin connected to the button
int maxPull = 60;          // Initial value for maxPull
int potValue = 0;          // Variable to store the potentiometer value
int mappedValue = 0;       // Variable to store the mapped value
bool settingsIsActive = false;     // Flag to track the state of the potentiometer

void setup() {
  Serial.begin(9600);     // Initialize serial communication
  pinMode(buttonPin, INPUT_PULLUP);  // Set button pin as input with internal pull-up resistor
}

void loop() {
  // Read the state of the button
  bool buttonState = digitalRead(buttonPin);
  
  // Toggle active/inactive state when button is pressed
  if (buttonState == LOW) {
    settingsIsActive = !SettingsIsActive;
    delay(250);  // Debouncing delay
  }
  
  // Update maxPull value only if potentiometer is active
  if (settingsIsActive) {
    // Read the value from the potentiometer
    potValue = analogRead(potPin);
    
    // Map the potentiometer value to the range of possible maxPull values
    mappedValue = map(potValue, 0, 1023, 0, 12);
    
    // Select the corresponding maxPull value from the list
    switch (mappedValue) {
      case 0:
        maxPull = 60;
        break;
      case 1:
        maxPull = 65;
        break;
      case 2:
        maxPull = 70;
        break;
      case 3:
        maxPull = 75;
        break;
      case 4:
        maxPull = 80;
        break;
      case 5:
        maxPull = 85;
        break;
      case 6:
        maxPull = 90;
        break;
      case 7:
        maxPull = 95;
        break;
      case 8:
        maxPull = 100;
        break;
      case 9:
        maxPull = 105;
        break;
      case 10:
        maxPull = 110;
        break;
      case 11:
        maxPull = 115;
        break;
      case 12:
        maxPull = 120;
        break;
    }
  }
  
  // Output the current maxPull value
  Serial.print("Max Pull: ");
  Serial.println(maxPull);
  Serial.print("kg");
  
  delay(100);  // Delay to stabilize readings
}

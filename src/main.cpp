/*
 * Author: Moniruzzaman Akash
 * Created on: Dec 07, 2024
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * Copyright (C) [2024] Moniruzzaman Akash
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <esp_system.h>
 #include <time.h>
 #include "esp_timer.h"
 #include "INA226.h"

 INA226 INA0(0x40);
 
 
 void restartEsp(void *arg) {
     // Serial.println("Restarting via Timer...");
     ESP.restart();
 }
 
 
 
 void publishBumpSensor();
 void publishChargingVoltage();
 void publishIRSensorWeight(float weight);
 void readIRSensors();
 float estimateDirection();
 float calculateMedian(float arr[], int size) ;
 float getVoltage();
 float getCurrent();
 float getPower();
 
 
 const int bumpPin = 27;
 const int voltageSensorPin = 34;
 
 
 // IR array variables
 const int numArray = 9;
 // Define pin numbers for the IR receivers
 const int irPins[] = {15, 25, 4, 16, 17, 5, 18, 19, 23};  // Array to hold the IR receiver pin numbers
 int irValues[numArray];  // Array to store the IR receiver values
 
 float direction = -1;
 float pre_direction = -1;
 float median = -1;
 const int medianWindowSize = 10; // Size of the moving window
 float directionHistory[medianWindowSize]; // Array to store the last 10 directions
 int historyIndex = 0; // Current index in the history array
 int historyCount = 0; // Number of values added so far
 
 
 // Float for Reference Voltage
 float ref_voltage = 3.3;
 
 //Last state saver
 unsigned long lastTimeStamp = 0;
 unsigned long last_charging_pub_TimeStamp = 0; //last timestamp velocity command received from ROS
 unsigned long last_bump_pub_TimeStamp = 0; //last timestamp battery status published
 unsigned long lastSuccessfulComm = 0; 
 
 
 struct timespec ts; //to get current timestamp
 
 int bumpState = 0;
 float voltage = 0;
 float current = 0;
 float power   = 0;
 int zeroCount = 0;
 void setup(void)
 {
   Serial.begin(115200); //Feedback over Serial Monitor and main controller
   
   pinMode(bumpPin, INPUT_PULLUP);
   pinMode(voltageSensorPin, INPUT);
 
   // IR array setup
   // Initialize the IR receiver pins as input using a for loop
   for (int i = 0; i < numArray; i++) {
     pinMode(irPins[i], INPUT);
   }

 
   // Initialize the direction history array with -1 (no signal detected)
   for (int i = 0; i < medianWindowSize; i++) {
     directionHistory[i] = -1;
   }
 
  // Setup INA226=====
  Serial.print("INA226_LIB_VERSION: ");
  Serial.println(INA226_LIB_VERSION);

  Wire.begin();
  if (!INA0.begin() )
  {
    Serial.println("INA0 could not connect. Fix and Reboot");
  }
  INA0.setMaxCurrentShunt(1, 0.002);


   // Create a hardware timer to trigger every 10 minutes
   esp_timer_create_args_t timer_args = {};
   timer_args.callback = &restartEsp;
   esp_timer_handle_t timer;
 
   esp_timer_create(&timer_args, &timer);
   esp_timer_start_once(timer, 600000000); // 10 minutes in microseconds
 
 } //end setup
 
 void loop()
 {
   if (!Serial) {  
     delay(1000);  // Wait before retrying
     esp_restart();  // Reset ESP32 if no serial connection is found
   }
 
  // Read values from the IR receivers
  readIRSensors();
 
   // Process the sensor values to estimate the direction
   direction = estimateDirection();
   
   if(median > -1){
    if(direction == -1){
      direction = pre_direction;
    }
   }

   // Add the new direction to the history
   directionHistory[historyIndex] = direction;
   historyIndex = (historyIndex + 1) % medianWindowSize; // Update the index circularly
   historyCount = min(historyCount + 1, medianWindowSize); // Keep track of the number of values
 
   // Calculate and print the moving median
   median = calculateMedian(directionHistory, historyCount);
 
   if(millis() - last_bump_pub_TimeStamp > 100){  //Report every 200ms
     bumpState = (int)!digitalRead(bumpPin);
     voltage = getVoltage();
     current = getCurrent();
     power = getPower();

     last_bump_pub_TimeStamp = millis();

     // If current is 0 for 5 times, restart the ESP32
      if (current == 0) {
        zeroCount++;
        if (zeroCount >= 5) {
          zeroCount = 0; // Reset the count
          Serial.println("Restarting ESP32 for zero current...");
          esp_restart();
        }
      }
 
     // Format message as a single `const char` string
     char message[50];  // Adjust size if needed
     snprintf(message, sizeof(message), "B%d,W%.2f,V%.2f,C%.4f\n", bumpState, median, voltage, current);
 
     // Send the formatted message
     Serial.write(message, strlen(message));
 }
 
 
   // Delay for a short period before the next reading
   delay(15); // Adjust delay as needed

  if(direction != 1) pre_direction = direction; // Previous valid sensor value
   
 
   
 }//end void loop()
 
void readIRSensors(){
  // Read values from the IR receivers and store in the array using a for loop
  for (int i = 0; i < numArray; i++) {
    irValues[i] = !digitalRead(irPins[i]);
    // Serial.print(irValues[i]);
    // Serial.print(",");
  }
  // Serial.println();
}
 
float getVoltage(){
   float voltage_ = INA0.getBusVoltage();
   return voltage_;
 }
 
float getCurrent(){
  float current_ = INA0.getCurrent_mA()/1000.0;
  
  return current_;
}

float getPower(){
  float power_ = INA0.getPower_mW();
  return power_;
}

 // Function to estimate the direction based on active sensors
 float estimateDirection() {
   int activeSensors = 0;    // Count of active sensors
   float weightedSum = 0.0;  // Weighted sum of active sensor indices
 
   for (int i = 0; i < numArray; i++) {
     if (irValues[i] == 1) { // Sensor detects signal
       activeSensors++;
       weightedSum += i; // Add the index to the weighted sum
     }
   }
 
   if (activeSensors == 0) {
     // No IR signal detected
     return -1;
   }
 
   // Compute average index for direction
   float averageIndex = weightedSum / activeSensors;
 
   return averageIndex; // Return the estimated direction (index-based)
 }
 
 // Function to calculate the median of the last N values
 float calculateMedian(float arr[], int size) {
   float sorted[size];
   int validCount = 0;
 
   // Filter out invalid values (-1) and copy valid values to a temporary array
   for (int i = 0; i < size; i++) {
     if (arr[i] != -1) {
       sorted[validCount++] = arr[i];
     }
   }
 
   if (validCount == 0) {
     // No valid values in the array
     return -1;
   }
 
   // Sort the valid values
   for (int i = 0; i < validCount - 1; i++) {
     for (int j = i + 1; j < validCount; j++) {
       if (sorted[i] > sorted[j]) {
         float temp = sorted[i];
         sorted[i] = sorted[j];
         sorted[j] = temp;
       }
     }
   }
 
   // Calculate the median
   if (validCount % 2 == 0) {
     return (sorted[validCount / 2 - 1 + 2] + sorted[validCount / 2 + 2]) / 2.0;
   } else {
     return sorted[validCount / 2 + 2];
   }
 }
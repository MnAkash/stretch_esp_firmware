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
 #include <esp_system.h>
 
 #include <time.h>
 #include "esp_timer.h"
 
 void restartEsp(void *arg) {
     // Serial.println("Restarting via Timer...");
     ESP.restart();
 }
 
 
 
 void publishBumpSensor();
 void publishChargingVoltage();
 void publishIRSensorWeight(float weight);
 float estimateDirection();
 float calculateMedian(float arr[], int size) ;
 float getVoltage();
 
 
 const int bumpPin = 27;
 const int voltageSensorPin = 34;
 
 
 // IR array variables
 const int numArray = 9;
 // Define pin numbers for the IR receivers
 const int irPins[] = {15, 2, 4, 16, 17, 5, 18, 19, 23};  // Array to hold the IR receiver pin numbers
 int irValues[numArray];  // Array to store the IR receiver values
 
 const int medianWindowSize = 10; // Size of the moving window
 float directionHistory[medianWindowSize]; // Array to store the last 10 directions
 int historyIndex = 0; // Current index in the history array
 int historyCount = 0; // Number of values added so far
 
 
 
 // Floats for ADC voltage & Input voltage
 float adc_voltage = 0.0;
 float in_voltage = 0.0;
  
 // Floats for resistor values in divider (in ohms)
 float R1 = 30000.0;
 float R2 = 7500.0; 
  
 // Float for Reference Voltage
 float ref_voltage = 3.3;
  
 // Integer for ADC value
 int adc_value = 0;
 
 //Last state saver
 unsigned long lastTimeStamp = 0;
 unsigned long last_charging_pub_TimeStamp = 0; //last timestamp velocity command received from ROS
 unsigned long last_bump_pub_TimeStamp = 0; //last timestamp battery status published
 unsigned long lastSuccessfulComm = 0; 
 
 
 struct timespec ts; //to get current timestamp
 
 int bumpState = 0;
 float voltage = 0;
 
 void setup(void)
 {
   Serial.begin(115200); //Feedback over Serial Monitor and main controller
   
   pinMode(bumpPin, INPUT_PULLUP);
   pinMode(voltageSensorPin, INPUT);
   pinMode(2, OUTPUT);
 
   // IR array setup
   // Initialize the IR receiver pins as input using a for loop
   for (int i = 0; i < numArray; i++) {
     pinMode(irPins[i], INPUT);
   }
 
   // Initialize the direction history array with -1 (no signal detected)
   for (int i = 0; i < medianWindowSize; i++) {
     directionHistory[i] = -1;
   }
 
 
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
     digitalWrite(2,HIGH);
     delay(50);
     digitalWrite(2,LOW);
     esp_restart();  // Reset ESP32 if no serial connection is found
   }
 
   // Read values from the IR receivers and store in the array using a for loop
   for (int i = 0; i < numArray; i++) {
     irValues[i] = !digitalRead(irPins[i]);
   }
 
   // Process the sensor values to estimate the direction
   float direction = estimateDirection();
 
   // Add the new direction to the history
   directionHistory[historyIndex] = direction;
   historyIndex = (historyIndex + 1) % medianWindowSize; // Update the index circularly
   historyCount = min(historyCount + 1, medianWindowSize); // Keep track of the number of values
 
   // Calculate and print the moving median
   float median = calculateMedian(directionHistory, historyCount);
 
   if(millis() - last_bump_pub_TimeStamp > 100){  //Report every 200ms
     bumpState = (int)!digitalRead(bumpPin);
     voltage = getVoltage();
     last_bump_pub_TimeStamp = millis();
 
     // Format message as a single `const char` string
     char message[50];  // Adjust size if needed
     snprintf(message, sizeof(message), "B%d,W%.2f,V%.2f\n", bumpState, median, voltage);
 
     // Send the formatted message
     Serial.write(message, strlen(message));
 }
 
 
   // Delay for a short period before the next reading
   delay(15); // Adjust delay as needed
 
   
 
   
 }//end void loop()
 
 
 float getVoltage(){
   // Read the Analog Input
   adc_value = analogRead(voltageSensorPin);
   adc_value += analogRead(voltageSensorPin); //take 2nd reading
 
   adc_value = adc_value / 2; //Take average
   
   // Determine voltage at ADC input
   adc_voltage  = (adc_value * ref_voltage) / 4096.0;
   
   // Calculate voltage at divider input
   in_voltage = adc_voltage*(R1+R2)/R2;
 
   return in_voltage;
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
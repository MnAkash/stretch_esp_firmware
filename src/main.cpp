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

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <time.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>


#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void publishBumpSensor();
void publishChargingVoltage();
void publishIRSensorWeight(float weight);
float estimateDirection();
float calculateMedian(float arr[], int size) ;


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

//Micro ROS variables
// rcl_subscription_t cmd_vel_sub;

rcl_publisher_t bump_publisher;
rcl_publisher_t charging_publisher;
rcl_publisher_t ir_publisher;

// geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int64 bump_msg;
std_msgs__msg__Float32 charging_voltage_msg;
std_msgs__msg__Float32 ir_sensor_msg;
std_msgs__msg__Int32 Int32_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node;
rcl_timer_t timer;

struct timespec ts; //to get current timestamp


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

  // ============================ microROS setup ============================
  // ====== Initialize microROS ======
  set_microros_serial_transports (Serial);
  delay(1000);
  allocator = rcl_get_default_allocator();
  
  // Initialize and modify options (Set DOMAIN ID to 25)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 25);

  // ===Wait for agent to be avialable
  bool is_connected = false;
  // Initialize micro-ROS session and communication
  while (!is_connected) {
    // Initialize the ROS 2 client
    rcl_ret_t ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    if (ret == RCL_RET_OK) {
      is_connected = true;
      Serial.println("Successfully connected to ROS 2 Agent.");
    } else {
      Serial.println("Failed to connect to ROS 2 Agent. Retrying...");
      delay(2000);  // Wait before retrying
    }
  }

  // ===Initialize after agent found
  // rclc_support_init(&support, 0, NULL, &allocator); //create init_options
  rclc_node_init_default(&node, "esp32_node", "", &support);
  // rclc_node_init_default(&node, "esp32_node", "", &support); // create node

  // ====== Create Publishers =======
  rcl_ret_t rc_a = rclc_publisher_init_default(
                &bump_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
                "/bump");

  rcl_ret_t rc_b = rclc_publisher_init_default(
                &charging_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                "/charging_voltage");

  rcl_ret_t rc_c = rclc_publisher_init_default(
                &ir_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                "/docking/ir_weight");

} //end setup

void loop()
{
  // Get the current time
  // clock_gettime(CLOCK_REALTIME, &ts);
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  
  if(millis() - last_bump_pub_TimeStamp > 100){  //Report every 200ms
      publishBumpSensor();
  }
  if(millis() - last_charging_pub_TimeStamp > 100){  //Report every 200ms
      publishChargingVoltage();
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
  if (median == -1) {
    // Serial.println("No valid median (insufficient data or no signal)");
    publishIRSensorWeight(-1.0);
  } else {
    // Serial.println(median, 3); // Print with 3 decimal places
    publishIRSensorWeight(median);
  }

  // Delay for a short period before the next reading
  delay(15); // Adjust delay as needed


  
}//end void loop()


void publishBumpSensor() {
  bump_msg.data = (int)!digitalRead(bumpPin);
  
  RCSOFTCHECK(rcl_publish(&bump_publisher, &bump_msg, NULL));
  last_bump_pub_TimeStamp = millis();
}

void publishIRSensorWeight(float weight) {
  ir_sensor_msg.data = weight;
  
  RCSOFTCHECK(rcl_publish(&ir_publisher, &ir_sensor_msg, NULL));
}

void publishChargingVoltage() {
  // Read the Analog Input
  adc_value = analogRead(voltageSensorPin);
  adc_value += analogRead(voltageSensorPin); //take 2nd reading

  adc_value = adc_value / 2; //Take average
  
  // Determine voltage at ADC input
  adc_voltage  = (adc_value * ref_voltage) / 4096.0;
  
  // Calculate voltage at divider input
  in_voltage = adc_voltage*(R1+R2)/R2;

  charging_voltage_msg.data = in_voltage;
  
  RCSOFTCHECK(rcl_publish(&charging_publisher, &charging_voltage_msg, NULL));
  last_charging_pub_TimeStamp = millis();
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
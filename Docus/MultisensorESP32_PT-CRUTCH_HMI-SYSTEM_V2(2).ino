// COMPLETE CODE: High-Performance ESP32 with Fixed Force Sensor Implementation
// Fixed HX711 implementation based on working simple code + 30Hz operation

#include <HX711.h>
#include "BluetoothSerial.h"
#include "esp_system.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <EEPROM.h>
#include <Wire.h>

//---- System Configuration ----
String file_name = "MultisensorESP32_PT - CRUTCH HMI SYSTEM - FIXED FORCE";

//---- HMI Configuration ----
#define ANALOG_PIN 33
#define ANALOG_CENTER_VALUE 2048
#define ANALOG_TOLERANCE_PERCENT 60
#define ANALOG_THRESHOLD_LOW (ANALOG_CENTER_VALUE - (ANALOG_CENTER_VALUE * ANALOG_TOLERANCE_PERCENT / 100))
#define ANALOG_THRESHOLD_HIGH (ANALOG_CENTER_VALUE + (ANALOG_CENTER_VALUE * ANALOG_TOLERANCE_PERCENT / 100))

#define B1_WAIT_TIME_MS 5000  // 5 seconds wait after B1 command
#define CALIBRATION_MIN_WAIT_MS 2000  // 2 seconds minimum wait during calibration
#define CALIBRATION_WEIGHT_GRAMS 900.0  // 0.9 kg = 900 grams

//---- High-Performance Frequency Configuration ----
#define IMU_UPDATE_RATE_HZ 200         // Excellent Madgwick performance
#define FORCE_UPDATE_RATE_HZ 30        // 30Hz with 3-reading average (reliable timing)
#define PRINT_RATE_HZ 20               // Output frequency
#define ANALOG_UPDATE_RATE_HZ 50       // Analog input sampling

#define IMU_PERIOD_MS (1000 / IMU_UPDATE_RATE_HZ)      // 5ms
#define FORCE_PERIOD_MS (1000 / FORCE_UPDATE_RATE_HZ)  // 33.33ms  
#define PRINT_PERIOD_MS (1000 / PRINT_RATE_HZ)         // 50ms
#define ANALOG_PERIOD_MS (1000 / ANALOG_UPDATE_RATE_HZ) // 20ms

//---- Communication Optimization ----
#define SERIAL_BAUD_RATE 921600        // 8x faster than 115200
#define USE_COMPACT_FORMAT true        // Compact message format
#define USE_QUATERNIONS true           // Use quaternions vs Euler

//---- Force Sensor Configuration ----
#define FORCE_AVERAGE_READINGS 3       // Average 3 readings per measurement
#define FORCE_ERROR_THRESHOLD 1000000  // Force validation
#define FORCE_MAX_NOT_READY_COUNT 1000 // Max consecutive not-ready counts before error

//---- Bluetooth Configuration ----
#define BT_PRINT
#define BETTER_PLOTTER

#ifdef BT_PRINT
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error ERRO 101: Bluetooth não está habilitado!
  #endif
  #if !defined(CONFIG_BT_SPP_ENABLED)
    #error ERRO 102: Bluetooth Serial não disponível!
  #endif
  BluetoothSerial SerialBT;
#endif

//---- Sensor Hardware ----
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "NXP_FXOS_FXAS.h"

Adafruit_Madgwick filter;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

//---- Hardware Pins ----
const int keyPadPins[] = { 4, 0, 15, 2 }; // B1, B2, B3, B4 (reordered)
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;
uint8_t loopDelay = 1;

//---- Force Sensor (Simplified Implementation) ----
HX711 scale;
uint8_t dataPin = 34;
uint8_t clockPin = 23;
uint8_t readingsNo = FORCE_AVERAGE_READINGS;  // 3 readings for averaging
const int calibrationAddress = 321;

//---- Enhanced State Machine ----
enum SystemState {
  STATE_IDLE,                    // Waiting for B1
  STATE_MOTOR_CALIBRATION,       // B1 pressed - calibrating motors
  STATE_NORMAL_OPERATION,        // Normal sensor streaming
  STATE_FORCE_CALIBRATION_INIT,  // B2 pressed - starting force calibration
  STATE_FORCE_CALIBRATION_TARE,  // Waiting for B3 to tare
  STATE_FORCE_CALIBRATION_SCALE, // Waiting for scale calculation
  STATE_EMERGENCY_STOP           // Emergency stop activated
};

SystemState current_system_state = STATE_IDLE;

//---- Analog Input State ----
enum AnalogState {
  ANALOG_W,   // Walking (< threshold_low)
  ANALOG_S,   // Standing (between thresholds)  
  ANALOG_ES   // Emergency Stop (> threshold_high)
};

AnalogState current_analog_state = ANALOG_S;
bool emergency_stop_latched = false;

//---- Device-Specific Configuration ----
bool analogInputDisabled = false;
bool can_send_data = false;

//---- Timing Variables ----
uint32_t imu_timestamp = 0;
uint32_t force_timestamp = 0;
uint32_t print_timestamp = 0;
uint32_t analog_timestamp = 0;
uint32_t state_change_timestamp = 0;

//---- Sensor Data Variables ----
float current_roll = 0, current_pitch = 0, current_yaw = 0;
float current_qw = 0, current_qx = 0, current_qy = 0, current_qz = 0;
float gx = 0, gy = 0, gz = 0;
bool imu_data_ready = false;

float current_force = 0;
bool force_data_ready = false;
uint32_t force_not_ready_count = 0;
uint32_t force_error_count = 0;

int current_analog_value = ANALOG_CENTER_VALUE;
String current_analog_command = "S";

//---- Performance Monitoring ----
uint32_t imu_cycles = 0;
uint32_t force_cycles = 0;
uint32_t print_cycles = 0;
uint32_t loop_max_time = 0;

//---- Filter Debug Variables ----
uint32_t filter_update_count = 0;
bool filter_converged = false;

//---- High-Performance Communication Queues ----
#define OUTPUT_BUFFER_SIZE 48
#define MAX_QUEUED_MESSAGES 30
#define BT_BURST_SIZE 5

struct UARTQueue {
  char messages[MAX_QUEUED_MESSAGES][OUTPUT_BUFFER_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
  uint32_t dropped_messages;
  uint32_t total_sent;
  uint32_t bytes_sent;
};

UARTQueue uart_queue = {0};
UARTQueue bt_queue = {0};

//---- Bluetooth Performance Stats ----
struct BluetoothStats {
  uint32_t messages_sent;
  uint32_t bytes_sent;
  uint32_t start_time;
  float current_bps;
  float peak_bps;
  uint32_t congestion_events;
};

BluetoothStats bt_stats = {0};

//==== ANALOG INPUT PROCESSING ====
void processAnalogInput() {
  current_analog_value = analogRead(ANALOG_PIN);
  
  AnalogState new_analog_state;
  
  if (current_analog_value < ANALOG_THRESHOLD_LOW) {
    new_analog_state = ANALOG_W; 
    current_analog_command = "W";
  } else if (current_analog_value > ANALOG_THRESHOLD_HIGH) {
    new_analog_state = ANALOG_ES;
    current_analog_command = "ES";
    // Latch emergency stop
    if (!emergency_stop_latched) {
      emergency_stop_latched = true;
      current_system_state = STATE_EMERGENCY_STOP;
      // Send ES message immediately
      char es_msg[OUTPUT_BUFFER_SIZE];
      snprintf(es_msg, sizeof(es_msg), "%lu,ES,AN:%d\n", millis(), current_analog_value);
      addToQueue(&uart_queue, es_msg);
      addToQueue(&bt_queue, es_msg);
    }
  } else {
    new_analog_state = ANALOG_S;
    current_analog_command = "S";
  }
  
  if (new_analog_state != current_analog_state) {
    current_analog_state = new_analog_state;
  }
}

//==== ENHANCED BUTTON STATE MACHINE ====
void handleEnhancedButtonStates() {
  bool button_pressed[numButtons];
  
  // Read all buttons
  for (int i = 0; i < numButtons; i++) {
    button_pressed[i] = (digitalRead(keyPadPins[i]) == LOW);
  }
  
  // Handle button presses based on current system state
  switch (current_system_state) {
    case STATE_IDLE:
      if (button_pressed[0]) { // B1 - Calibrate motor ranges
        current_system_state = STATE_MOTOR_CALIBRATION;
        state_change_timestamp = millis();
        char mc_msg[OUTPUT_BUFFER_SIZE];
        snprintf(mc_msg, sizeof(mc_msg), "%lu,MC,AN:%d\n", millis(), current_analog_value);
        addToQueue(&uart_queue, mc_msg);
        addToQueue(&bt_queue, mc_msg);
        delay(300); // Debounce
      }
      if (button_pressed[1]) { // B2 - Force calibration (also from IDLE)
        current_system_state = STATE_FORCE_CALIBRATION_INIT;
        state_change_timestamp = millis();
        char fc_msg[OUTPUT_BUFFER_SIZE];
        snprintf(fc_msg, sizeof(fc_msg), "%lu,FC\n", millis());
        addToQueue(&uart_queue, fc_msg);
        addToQueue(&bt_queue, fc_msg);
        blinkLed(2, 150);
        delay(2000);
        blinkLed(2, 150);
        addToQueue(&uart_queue, "FORCE CALIBRATION: Press B3 for TARE (no load)\n");
        addToQueue(&bt_queue, "FORCE CALIBRATION: Press B3 for TARE (no load)\n");
        delay(300); // Debounce
      }
      break;
      
    case STATE_MOTOR_CALIBRATION:
      // Wait for the specified time before transitioning to normal operation
      if ((millis() - state_change_timestamp) >= B1_WAIT_TIME_MS) {
        current_system_state = STATE_NORMAL_OPERATION;
        can_send_data = true; // Enable data stream
        addToQueue(&uart_queue, "NORMAL OPERATION STARTED\n");
        addToQueue(&bt_queue, "NORMAL OPERATION STARTED\n");
      }
      break;
      
    case STATE_NORMAL_OPERATION:
      if (button_pressed[1]) { // B2 - Force calibration (during normal operation or idle)
        current_system_state = STATE_FORCE_CALIBRATION_INIT;
        state_change_timestamp = millis();
        char fc_msg[OUTPUT_BUFFER_SIZE];
        snprintf(fc_msg, sizeof(fc_msg), "%lu,FC,%d,%d,%d,%d,%d\n",
                 millis(),
                 (int)(current_force * 100),
                 (int)(current_qw * 1000),
                 (int)(current_qx * 1000),
                 (int)(current_qy * 1000),
                 (int)(current_qz * 1000));
        addToQueue(&uart_queue, fc_msg);
        addToQueue(&bt_queue, fc_msg);
        blinkLed(2, 150);
        delay(2000);
        blinkLed(2, 150);
        addToQueue(&uart_queue, "FORCE CALIBRATION: Press B3 for TARE (no load)\n");
        addToQueue(&bt_queue, "FORCE CALIBRATION: Press B3 for TARE (no load)\n");
        delay(300); // Debounce
      }
      if (button_pressed[2]) { // B3 - TARE during calibration / Print analog during operation
        if (!analogInputDisabled) {
          char analog_msg[32];
          snprintf(analog_msg, sizeof(analog_msg), "AN:%d\n", current_analog_value);
          addToQueue(&uart_queue, analog_msg);
          addToQueue(&bt_queue, analog_msg);
        }
        delay(300); // Debounce
      }
      break;
      
    case STATE_FORCE_CALIBRATION_INIT:
      // Wait for B3 press
      if (button_pressed[2]) { // B3 - Perform TARE (no load)
        current_system_state = STATE_FORCE_CALIBRATION_TARE;
        state_change_timestamp = millis();
        
        // LED sequence for tare
        blinkLed(3, 150);
        
        // Perform tare with NO LOAD
        addToQueue(&uart_queue, "TARING with NO LOAD...\n");
        addToQueue(&bt_queue, "TARING with NO LOAD...\n");
        scale.tare(20);
        
        blinkLed(3, 150);
        
        addToQueue(&uart_queue, "TARE COMPLETE - Now add 900g weight and wait 3 seconds\n");
        addToQueue(&bt_queue, "TARE COMPLETE - Now add 900g weight and wait 3 seconds\n");
        
        delay(300); // Debounce
      }
      break;
      
    case STATE_FORCE_CALIBRATION_TARE:
      if ((millis() - state_change_timestamp) >= 3000) { // Wait 3 seconds for user to add weight
        // Automatically calculate scale factor
        current_system_state = STATE_FORCE_CALIBRATION_SCALE;
        
        addToQueue(&uart_queue, "CALCULATING SCALE with 900g weight...\n");
        addToQueue(&bt_queue, "CALCULATING SCALE with 900g weight...\n");
        
        // Use the HX711 library's calibrate_scale method
        scale.calibrate_scale(CALIBRATION_WEIGHT_GRAMS, 20);
        
        float scale_factor = scale.get_scale();
        float offset_value = scale.get_offset();
        
        if (scale_factor != 0 && !isnan(scale_factor) && !isinf(scale_factor)) {
          writeCalibrationToEEPROM(scale_factor, offset_value);
          char cal_msg[150];
          snprintf(cal_msg, sizeof(cal_msg), "CALIBRATION SUCCESS!\nScale: %.6f\nOffset: %.2f\n", 
                   scale_factor, offset_value);
          addToQueue(&uart_queue, cal_msg);
          addToQueue(&bt_queue, cal_msg);
          
          delay(1000);
          if (scale.is_ready()) {
            float test_weight = scale.get_units(10);
            char test_msg[80];
            snprintf(test_msg, sizeof(test_msg), "Test reading: %.2f g\n", test_weight);
            addToQueue(&uart_queue, test_msg);
            addToQueue(&bt_queue, test_msg);
          }
        } else {
          addToQueue(&uart_queue, "CALIBRATION FAILED - Invalid scale factor\n");
          addToQueue(&bt_queue, "CALIBRATION FAILED - Invalid scale factor\n");
          char error_msg[100];
          snprintf(error_msg, sizeof(error_msg), "Scale factor: %.6f\n", scale_factor);
          addToQueue(&uart_queue, error_msg);
        }
        
        current_system_state = STATE_NORMAL_OPERATION;
        addToQueue(&uart_queue, "Remove weight. Returning to NORMAL OPERATION\n");
        addToQueue(&bt_queue, "Remove weight. Returning to NORMAL OPERATION\n");
      }
      break;
      
    case STATE_EMERGENCY_STOP:
      if (button_pressed[3]) { // B4 - Reset from emergency stop (pin 2)
        emergency_stop_latched = false;
        current_system_state = STATE_IDLE;
        current_analog_command = "S"; // Reset to standing
        addToQueue(&uart_queue, "EMERGENCY STOP RESET - System returned to IDLE\n");
        addToQueue(&bt_queue, "EMERGENCY STOP RESET - System returned to IDLE\n");
        delay(300); // Debounce
      }
      break;
  }
}

//==== SIMPLIFIED FORCE PROCESSING (Based on Working Code) ====
void processForce() {
  // Only process force during normal operation or calibration
  if (current_system_state != STATE_NORMAL_OPERATION && 
      current_system_state != STATE_FORCE_CALIBRATION_INIT &&
      current_system_state != STATE_FORCE_CALIBRATION_TARE &&
      current_system_state != STATE_FORCE_CALIBRATION_SCALE) {
    return;
  }
  
  force_cycles++;
  
  // Use the simple, reliable approach from the working code
  if (scale.is_ready()) {
    // This will take 3 readings and average them automatically
    // At 80Hz HX711 rate, 3 readings = ~37.5ms, fits in our 33ms period
    current_force = scale.get_units(readingsNo);
    
    // Validate reading
    if (abs(current_force) < FORCE_ERROR_THRESHOLD) {
      force_data_ready = true;
      force_not_ready_count = 0; // Reset not-ready counter
    } else {
      force_error_count++;
      // Keep last valid reading
    }
    
  } else {
    // Handle not-ready case (similar to working code)
    force_not_ready_count++;
    
    // Print error occasionally to avoid spam
    if (force_not_ready_count == FORCE_MAX_NOT_READY_COUNT) {
      char error_msg[100];
      snprintf(error_msg, sizeof(error_msg), 
               "Force sensor not ready (count: %lu, time: %lu ms)\n", 
               force_not_ready_count, millis());
      addToQueue(&uart_queue, error_msg);
      force_not_ready_count = 0; // Reset counter
    }
    
    // Still mark as ready to continue operation with last valid reading
    force_data_ready = true;
  }
}

//==== HIGH-PERFORMANCE COMMUNICATION ====
bool addToQueue(UARTQueue* queue, const char* message) {
  if (queue->count >= MAX_QUEUED_MESSAGES) {
    queue->dropped_messages++;
    return false;
  }
  
  strncpy(queue->messages[queue->head], message, OUTPUT_BUFFER_SIZE - 1);
  queue->messages[queue->head][OUTPUT_BUFFER_SIZE - 1] = '\0';
  
  queue->head = (queue->head + 1) % MAX_QUEUED_MESSAGES;
  queue->count++;
  queue->total_sent++;
  queue->bytes_sent += strlen(message);
  
  return true;
}

bool getFromQueue(UARTQueue* queue, char* buffer) {
  if (queue->count == 0) {
    return false;
  }
  
  strcpy(buffer, queue->messages[queue->tail]);
  queue->tail = (queue->tail + 1) % MAX_QUEUED_MESSAGES;
  queue->count--;
  
  return true;
}

void formatCompactMessage(char* buffer, size_t buffer_size) {
  snprintf(buffer, buffer_size, "%lu,%s,%d,%d,%d,%d,%d\n",
           millis(),
           current_analog_command.c_str(),
           (int)(current_force * 100),          // 2 decimal places
           (int)(current_qw * 1000),            // 3 decimal places  
           (int)(current_qx * 1000),
           (int)(current_qy * 1000),
           (int)(current_qz * 1000));
}


void updateBluetoothBurst() {
  static char buffer[OUTPUT_BUFFER_SIZE];
  static uint32_t last_stats_update = 0;
  
  #ifdef BT_PRINT
  if (SerialBT.hasClient()) {
    for (int i = 0; i < BT_BURST_SIZE && bt_queue.count > 0; i++) {
      if (getFromQueue(&bt_queue, buffer)) {
        SerialBT.print(buffer);
        bt_stats.messages_sent++;
        bt_stats.bytes_sent += strlen(buffer);
      }
    }
    
    if (millis() - last_stats_update > 1000) {
      uint32_t elapsed_ms = millis() - bt_stats.start_time;
      if (elapsed_ms > 0) {
        bt_stats.current_bps = (bt_stats.bytes_sent * 8.0 * 1000.0) / elapsed_ms;
        if (bt_stats.current_bps > bt_stats.peak_bps) {
          bt_stats.peak_bps = bt_stats.current_bps;
        }
      }
      last_stats_update = millis();
    }
  } else {
    if (bt_queue.count > 0) {
      bt_stats.congestion_events++;
      bt_queue.count = 0;
      bt_queue.head = bt_queue.tail = 0;
    }
  }
  #endif
}

void updateUARTFast() {
  static char buffer[OUTPUT_BUFFER_SIZE];
  
  #ifdef BETTER_PLOTTER
  for (int i = 0; i < 10 && uart_queue.count > 0; i++) {
    if (getFromQueue(&uart_queue, buffer)) {
      Serial.print(buffer);
    }
  }
  #endif
}

//==== MAIN PROCESS FUNCTIONS ====
void processIMU() {
  // Only process IMU during normal operation
  if (current_system_state != STATE_NORMAL_OPERATION) {
    return;
  }
  
  imu_cycles++;
  
  // BLOCKING VERSION - ensures reliable filter convergence
  sensors_event_t accel, gyro, mag;
  
  // Read all sensors
  if (accelerometer->getEvent(&accel) && 
      gyroscope->getEvent(&gyro) && 
      magnetometer->getEvent(&mag)) {
    
    // Calibrate readings
    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    // Convert gyroscope from Rad/s to Degrees/s
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update Madgwick filter with proper timing
    filter.update(gx, gy, gz,
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    // Get quaternion results
    #if USE_QUATERNIONS
      float new_qw, new_qx, new_qy, new_qz;
      filter.getQuaternion(&new_qw, &new_qx, &new_qy, &new_qz);
      
      // Check for filter convergence
      static float last_qw = 0, last_qx = 0, last_qy = 0, last_qz = 0;
      float quaternion_change = abs(new_qw - last_qw) + abs(new_qx - last_qx) + 
                               abs(new_qy - last_qy) + abs(new_qz - last_qz);
      
      current_qw = new_qw;
      current_qx = new_qx;
      current_qy = new_qy;
      current_qz = new_qz;
      
      filter_update_count++;
      
      // Check convergence after 100 updates
      if (!filter_converged && filter_update_count > 100) {
        if (quaternion_change < 0.005f) {
          filter_converged = true;
          addToQueue(&uart_queue, "FILTER: Converged successfully!\n");
        }
      }
      
      last_qw = new_qw;
      last_qx = new_qx;
      last_qy = new_qy;
      last_qz = new_qz;
      
    #else
      current_roll = filter.getRoll();
      current_pitch = filter.getPitch();
      current_yaw = filter.getYaw();
    #endif
    
    imu_data_ready = true;
  } else {
    // Sensor read failed
    static uint32_t read_failures = 0;
    read_failures++;
    if (read_failures % 100 == 0) {
      char error_msg[64];
      snprintf(error_msg, sizeof(error_msg), "IMU read failures: %lu\n", read_failures);
      addToQueue(&uart_queue, error_msg);
    }
  }
}

void processPrint() {
  // Only send data during normal operation
  if (current_system_state != STATE_NORMAL_OPERATION) {
    return;
  }
  
  print_cycles++;
  
  // Always format and send data
  static char output_buffer[OUTPUT_BUFFER_SIZE];
  formatCompactMessage(output_buffer, sizeof(output_buffer));
  
  addToQueue(&uart_queue, output_buffer);
  addToQueue(&bt_queue, output_buffer);
  
  // Performance stats less frequently
  if (print_cycles % 200 == 0) { // Every 10 seconds at 20Hz
    printAdvancedPerformanceStats();
  }
}

void printAdvancedPerformanceStats() {
  static char stats_buffer[200];
  
  uint32_t uptime_seconds = millis() / 1000;
  if (uptime_seconds > 0) {
    float real_imu_freq = (float)imu_cycles / uptime_seconds;
    float real_force_freq = (float)force_cycles / uptime_seconds;
    float real_print_freq = (float)print_cycles / uptime_seconds;
    float filter_freq = (float)filter_update_count / uptime_seconds;
    
    // SHORTER message to prevent truncation
    snprintf(stats_buffer, sizeof(stats_buffer),
             "PERF: IMU:%.0f Force:%.0f Print:%.0f Filter:%.0f State:%d Errors:%lu\n",
             real_imu_freq, real_force_freq, real_print_freq, filter_freq, 
             current_system_state, force_error_count);
    
    addToQueue(&uart_queue, stats_buffer);
    addToQueue(&bt_queue, stats_buffer);
    
    // Queue status
    if (analogInputDisabled) {
      snprintf(stats_buffer, sizeof(stats_buffer),
               "QUEUES: UART=%d BT=%d Dropped=%lu Analog=N/A Force=%.1f\n",
               uart_queue.count, bt_queue.count,
               uart_queue.dropped_messages + bt_queue.dropped_messages,
               current_force);
    } else {
      snprintf(stats_buffer, sizeof(stats_buffer),
               "QUEUES: UART=%d BT=%d Dropped=%lu Analog=%d Force=%.1f\n",
               uart_queue.count, bt_queue.count,
               uart_queue.dropped_messages + bt_queue.dropped_messages,
               current_analog_value, current_force);
    }
    
    addToQueue(&uart_queue, stats_buffer);
  }
  
  loop_max_time = 0;
}

//==== LED INDICATOR ====
void blinkLed(int count, int blink_delay_ms) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ledPin, HIGH);
    delay(blink_delay_ms);
    digitalWrite(ledPin, LOW);
    if (i < count - 1) {
      delay(blink_delay_ms);
    }
  }
}

void updateLedIndicator() {
  // Disable continuous blinking during calibration sequences
  if (current_system_state == STATE_FORCE_CALIBRATION_INIT ||
      current_system_state == STATE_FORCE_CALIBRATION_TARE ||
      current_system_state == STATE_FORCE_CALIBRATION_SCALE) {
    digitalWrite(ledPin, LOW); // Ensure LED is off when not in a blink sequence
    return;
  }

  static uint32_t last_blink_time = 0;
  uint32_t blink_period_ms = 1000; // Default to 1 Hz for IDLE

  // Bluetooth not connected: 5 Hz
  if (!SerialBT.hasClient()) {
    blink_period_ms = 200; // 1000ms / 5Hz = 200ms period
  } else {
    // Bluetooth connected, check system state
    switch (current_system_state) {
      case STATE_NORMAL_OPERATION:
        blink_period_ms = 100; // 1000ms / 10Hz = 100ms period
        break;
      case STATE_IDLE:
        blink_period_ms = 1000; // 1000ms / 1Hz = 1000ms period
        break;
      case STATE_EMERGENCY_STOP:
        blink_period_ms = 10000; // 10000ms / 0.1Hz = 10s period
        break;
      default:
        // For other states, let's use the IDLE blink rate
        blink_period_ms = 1000;
        break;
    }
  }

  // Toggle LED based on the calculated period
  if (millis() - last_blink_time >= (blink_period_ms / 2)) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    last_blink_time = millis();
  }
}


//==== MAIN LOOP ====
void loop() {
  uint32_t loop_start = micros();
  
  // Always handle buttons and analog input
  handleEnhancedButtonStates();

  if (!analogInputDisabled) {
    if ((millis() - analog_timestamp) >= ANALOG_PERIOD_MS) {
      processAnalogInput();
      analog_timestamp = millis();
    }
  }

  if ((millis() - imu_timestamp) >= IMU_PERIOD_MS) {
    processIMU();
    imu_timestamp = millis();
  }

  if ((millis() - force_timestamp) >= FORCE_PERIOD_MS) {
    processForce();
    force_timestamp = millis();
  }

  if ((millis() - print_timestamp) >= PRINT_PERIOD_MS) {
    processPrint();
    print_timestamp = millis();
  }

  updateBluetoothBurst();
  updateUARTFast();
  updateLedIndicator();

  uint32_t loop_time_us = micros() - loop_start;
  uint32_t loop_time_ms = loop_time_us / 1000;
  
  if (loop_time_ms > loop_max_time) {
    loop_max_time = loop_time_ms;
  }
  
  if (loop_time_us < 100) {
    delayMicroseconds(25);
  }
}

//==== SETUP ====
void setup() {
  #ifdef BETTER_PLOTTER
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setTxBufferSize(2048);
    while (!Serial) {}
    Serial.println(file_name);
    Serial.println("=== CRUTCH HMI SYSTEM WITH FIXED FORCE SENSOR ===");
    Serial.println("IMU: " + String(IMU_UPDATE_RATE_HZ) + "Hz (" + String(IMU_PERIOD_MS) + "ms)");
    Serial.println("Force: " + String(FORCE_UPDATE_RATE_HZ) + "Hz (" + String(FORCE_PERIOD_MS) + "ms)");
    Serial.println("Force Readings: " + String(FORCE_AVERAGE_READINGS) + " average per measurement");
    Serial.println("Print: " + String(PRINT_RATE_HZ) + "Hz (" + String(PRINT_PERIOD_MS) + "ms)");
    if (!analogInputDisabled) {
      Serial.println("Analog: " + String(ANALOG_UPDATE_RATE_HZ) + "Hz (" + String(ANALOG_PERIOD_MS) + "ms)");
      Serial.println("Analog Pin: " + String(ANALOG_PIN));
      Serial.println("Analog Thresholds: Low=" + String(ANALOG_THRESHOLD_LOW) + " High=" + String(ANALOG_THRESHOLD_HIGH));
    }
    Serial.println("Calibration Weight: " + String(CALIBRATION_WEIGHT_GRAMS) + "g");
    Serial.println("Message format: ENHANCED COMPACT with analog commands");
    Serial.println("IMU Processing: BLOCKING (for reliability)");
    Serial.println("Force Processing: SIMPLIFIED (based on working code)");
    Serial.println("System State: IDLE (waiting for B1)");
    Serial.println("=======================================");
  #endif

  // Setup analog input pin
  if (!analogInputDisabled) {
    pinMode(ANALOG_PIN, INPUT);
  }

  for (int i = 0; i < numButtons; i++) {
    pinMode(keyPadPins[i], INPUT_PULLUP);
  }
  pinMode(ledPin, OUTPUT);

  #ifdef BT_PRINT
    uint64_t chipid = ESP.getEfuseMac();
    String chipIdString = String((uint16_t)(chipid >> 32), HEX);
    
    if (chipIdString.equalsIgnoreCase("74d3")) {
      analogInputDisabled = true;
    }

    String deviceName = "CrutchHMI-BT-" + chipIdString;
    SerialBT.begin(deviceName);
    bt_stats.start_time = millis();
    Serial.println("Bluetooth device: " + deviceName);
    if (analogInputDisabled) {
      Serial.println("!!! SPECIAL DEVICE (74d3): ANALOG INPUT DISABLED !!!");
    }
  #endif

  if (!cal.begin()) {
    printMessageLn("ERRO 201: Falha ao inicializar calibração");
  } else if (!cal.loadCalibration()) {
    printMessageLn("Nenhuma calibração encontrada");
  } else {
    printCalibration();
    delay(1000);
  }

  if (!init_sensors()) {
    printMessageLn("ERRO 202: Falha ao encontrar sensores");
    while (1) delay(10);
  }

  setup_sensors();
  
  // FIXED FILTER INITIALIZATION
  filter.begin(IMU_UPDATE_RATE_HZ);
  delay(100); // Give filter time to initialize properly
  printMessageLn("Filter initialized at " + String(IMU_UPDATE_RATE_HZ) + "Hz");
  
  Wire.setClock(400000);

  // Initialize HX711 with simplified setup (based on working code)
  scale.begin(dataPin, clockPin);
  scale.set_average_mode();  // Set to average mode like working code
  EEPROM.begin(512);

  loadFromMemory();  // Load calibration from EEPROM

  printMessageLn("---- HX711 Force Sensor ----");
  printMessage("MODE: ");
  printMessageLn(getModeNameHX711(scale.get_mode()));
  printMessage("GAIN: ");
  printMessageLn(getGainNameHX711(scale.get_gain()));
  printMessage("Readings per measurement: ");
  printMessageLn(String(readingsNo));
  printMessage("Expected reading time: ~");
  printMessageLn(String(readingsNo * 12.5) + "ms at 80Hz");

  // Initialize all timestamps
  imu_timestamp = millis();
  force_timestamp = millis();
  print_timestamp = millis();
  analog_timestamp = millis();
  
  // Print button instructions
  printMessageLn("=== BUTTON FUNCTIONS ===");
  printMessageLn("B1 (Pin 4): Calibrate motor ranges + start normal operation");
  printMessageLn("B2 (Pin 0): Start force sensor calibration sequence");
  printMessageLn("B3 (Pin 15): TARE during calibration / Print analog during operation");
  printMessageLn("B4 (Pin 2): Reset emergency stop (returns to IDLE)");
  if (!analogInputDisabled) {
    printMessageLn("=== ANALOG INPUT ===");
    printMessageLn("Pin 33: W (<" + String(ANALOG_THRESHOLD_LOW) + "), S (middle), ES (>" + String(ANALOG_THRESHOLD_HIGH) + ")");
  }
  printMessageLn("=== FORCE SENSOR (SIMPLIFIED) ===");
  printMessageLn("Average readings: " + String(readingsNo) + " samples per measurement");
  printMessageLn("Force frequency: " + String(FORCE_UPDATE_RATE_HZ) + "Hz");
  printMessageLn("Using simplified blocking approach (like working code)");
  printMessageLn("Library version: " + String(HX711_LIB_VERSION));
  printMessageLn("Initial scale: " + String(scale.get_scale()));
  printMessageLn("Initial offset: " + String(scale.get_offset()));
  
  // Test initial force reading
  if (scale.is_ready()) {
    float test_force = scale.get_units(5);
    printMessageLn("Initial force test: " + String(test_force) + " units");
  } else {
    printMessageLn("WARNING: Force sensor not ready at startup");
  }
  
  printMessageLn("========================");
  printMessageLn("=== CRUTCH HMI SYSTEM READY - PRESS B1 TO START ===");
}

//==== UTILITY FUNCTIONS ====
void printMessage(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.print(message);
    }
  #endif
  #ifdef BETTER_PLOTTER
    Serial.print(message);
  #endif
}

void printMessageLn(const String& message) {
  #ifdef BT_PRINT
    if (SerialBT.hasClient()) {
      SerialBT.println(message);
    }
  #endif
  #ifdef BETTER_PLOTTER
    Serial.println(message);
  #endif
}

void printCalibration() {
  printMessageLn("Calibração carregada com sucesso");
  printMessageLn("Calibrações encontradas: ");
  printMessage("\tOffset Magnético Rígido: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.mag_hardiron[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tOffset Magnético Suave: ");
  for (int i = 0; i < 9; i++) {
    printMessage(String(cal.mag_softiron[i]));
    if (i != 8) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tMagnitude do Campo Magnético: ");
  printMessageLn(String(cal.mag_field));
  printMessage("\tOffset de Taxa Zero do Giroscópio: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.gyro_zerorate[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
  printMessage("\tOffset Zero G do Acelerômetro: ");
  for (int i = 0; i < 3; i++) {
    printMessage(String(cal.accel_zerog[i]));
    if (i != 2) printMessage(", ");
  }
  printMessageLn("");
}

String getModeNameHX711(uint8_t mode) {
  switch (mode) {
    case HX711_AVERAGE_MODE: return "Modo Média";
    case HX711_MEDIAN_MODE: return "Modo Mediana";
    case HX711_MEDAVG_MODE: return "Modo MedAvg";
    case HX711_RUNAVG_MODE: return "Modo RunAvg";
    case HX711_RAW_MODE: return "Modo Bruto";
    default: return "Modo Desconhecido";
  }
}

String getGainNameHX711(uint8_t gain) {
  switch (gain) {
    case HX711_CHANNEL_A_GAIN_128: return "Canal A Ganho 128";
    case HX711_CHANNEL_A_GAIN_64: return "Canal A Ganho 64";
    case HX711_CHANNEL_B_GAIN_32: return "Canal B Ganho 32";
    default: return "Ganho Desconhecido";
  }
}

void readCalibrationFromEEPROM(float &scaleValue, float &offsetValue) {
  EEPROM.get(calibrationAddress, scaleValue);
  EEPROM.get(calibrationAddress + sizeof(float), offsetValue);
}

void writeCalibrationToEEPROM(float scaleValue, float offsetValue) {
  EEPROM.put(calibrationAddress, scaleValue);
  EEPROM.put(calibrationAddress + sizeof(float), offsetValue);
  EEPROM.commit();
  
  char cal_save_msg[100];
  snprintf(cal_save_msg, sizeof(cal_save_msg), "Calibration saved: Scale=%.6f, Offset=%.2f\n", 
           scaleValue, offsetValue);
  printMessageLn(String(cal_save_msg));
}

void loadFromMemory() {
  float scaleValue, offsetValue;
  readCalibrationFromEEPROM(scaleValue, offsetValue);

  // Check if EEPROM contains valid data
  if (isnan(scaleValue) || scaleValue == 0) {
    scaleValue = -5.752855;  // Default scale value from working code
    printMessageLn("Nenhuma Calibração Encontrada!");
    printMessageLn("Valor de Escala definido: " + String(scaleValue));
  } else {
    printMessageLn("Valor de Escala definido da Memória: " + String(scaleValue));
  }
  scale.set_scale(scaleValue);

  if (isnan(offsetValue)) {
    delay(500);
    scale.tare(20);  // Tare the scale with 20 readings
    delay(500);
    printMessageLn("Balança tarada!");
  } else {
    printMessageLn("Valor de Offset definido da Memória: " + String(offsetValue));
    scale.set_offset(offsetValue);
  }
  
  // Test reading to verify setup
  if (scale.is_ready()) {
    float test_units = scale.get_units(5);
    printMessageLn("Teste de leitura inicial: " + String(test_units) + " unidades");
  }
}

// Helper function to get system state name for debugging
String getSystemStateName(SystemState state) {
  switch(state) {
    case STATE_IDLE: return "IDLE";
    case STATE_MOTOR_CALIBRATION: return "MOTOR_CAL";
    case STATE_NORMAL_OPERATION: return "NORMAL";
    case STATE_FORCE_CALIBRATION_INIT: return "FORCE_CAL_INIT";
    case STATE_FORCE_CALIBRATION_TARE: return "FORCE_CAL_TARE";
    case STATE_FORCE_CALIBRATION_SCALE: return "FORCE_CAL_SCALE";
    case STATE_EMERGENCY_STOP: return "EMERGENCY";
    default: return "UNKNOWN";
  }
}

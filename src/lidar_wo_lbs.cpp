#include <Arduino.h>

#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

HardwareSerial LidarSerial(2);

// --- GLOBALS ---
float shadow_distance = 9999.0f;
float shadow_angle = 0.0f;

struct ObstacleData {
  float closest_distance;
  float closest_angle;
  unsigned long timestamp;
  ObstacleData() : closest_distance(9999.0f), closest_angle(0.0f) {} 
};

ObstacleData current_obstacle;
SemaphoreHandle_t obstacleMutex;


// --- LIDAR TASK (Core 0) ---
void LidarTask(void *pvParameters)
{
  Serial.println("Lidar Task Started. Waiting 3 seconds for Lidar to boot...");
  vTaskDelay(pdMS_TO_TICKS(3000)); 

  Serial.println("Sending Wake-up command...");
  uint8_t startMotorCmd[] = {0xA5, 0x20};
  LidarSerial.write(startMotorCmd, sizeof(startMotorCmd));

  Serial.println("Custom Hardware Parser Active! Scanning...");

  uint8_t packet[5];
  int packet_idx = 0;
  unsigned long last_data_time = millis();

  while (true)
  {
    bool got_data = false;

    // Read the raw hardware bytes directly
    while (LidarSerial.available())
    {
      got_data = true;
      uint8_t b = LidarSerial.read();

      // BYTE 0: Sync and Quality
      if (packet_idx == 0) {
        uint8_t sync = b & 0x03;
        if (sync == 0x01 || sync == 0x02) { // Valid RPLidar sync bits
          packet[0] = b;
          packet_idx = 1;
        }
      }
      // BYTE 1: Angle Checkbit
      else if (packet_idx == 1) {
        if ((b & 0x01) == 0x01) { // Checkbit must be 1
          packet[1] = b;
          packet_idx = 2;
        } else {
          packet_idx = 0; // Invalid, reset
          
          // Re-evaluate current byte just in case it's actually Byte 0
          uint8_t sync = b & 0x03;
          if (sync == 0x01 || sync == 0x02) {
            packet[0] = b;
            packet_idx = 1;
          }
        }
      }
      // BYTES 2, 3, 4: Data
      else {
        packet[packet_idx] = b;
        packet_idx++;

        // WE HAVE A FULL 5-BYTE PACKET!
        if (packet_idx == 5) {
          packet_idx = 0; // Reset for the next packet

          // Decode the math exactly how the C1 transmits it
          bool startBit = (packet[0] & 0x01) == 1;
          byte quality = packet[0] >> 2;
          float angle_deg = ((packet[2] << 7) | (packet[1] >> 1)) / 64.0f;
          float distance_mm = ((packet[4] << 8) | packet[3]) / 4.0f;

          // --- YOUR QUADRUPED LOGIC ---
          
          // If the Lidar completed a full 360-degree rotation
          if(startBit) {
            if(xSemaphoreTake(obstacleMutex, portMAX_DELAY) == pdTRUE) {
              current_obstacle.closest_distance = shadow_distance;
              current_obstacle.closest_angle = shadow_angle;
              current_obstacle.timestamp = millis();
              xSemaphoreGive(obstacleMutex);
            }
            shadow_distance = 9999.0f;
            shadow_angle = 0.0f;
          } 

          // Filter for objects directly in front of the robot (0-30 deg & 330-360 deg)
          if (quality > 0 && distance_mm > 0 && distance_mm <= 1000.0f) {
            if (angle_deg <= 30.0f || angle_deg >= 330.0f) {
              if(distance_mm < shadow_distance) {
                shadow_distance = distance_mm;
                shadow_angle = angle_deg;
              }
            }
          }
        }
      }
    }

    // Keep the motor alive if the Lidar gets stuck
    if (got_data) {
      last_data_time = millis();
    } else {
      if (millis() - last_data_time > 1500) {
        LidarSerial.write(startMotorCmd, sizeof(startMotorCmd));
        last_data_time = millis();
      }
    }

    // Pet the Watchdog
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}


// --- WALKING TASK (Core 1) ---
enum RobotState { STATE_WALKING_FORWARD, STATE_STOPPED, STATE_SKID_TURN };
RobotState current_state = STATE_STOPPED;

void WalkingTask(void *pvParameters)
{
  unsigned long last_print_time = 0;

  while(true)
  {
    float my_distance = 9999.0f;
    float my_angle = 0.0f;
    unsigned long my_timestamp = 0;

    if (xSemaphoreTake(obstacleMutex,portMAX_DELAY) == pdTRUE) {
      my_distance = current_obstacle.closest_distance;
      my_angle = current_obstacle.closest_angle;
      my_timestamp = current_obstacle.timestamp;
      xSemaphoreGive(obstacleMutex);
    }

    if( millis() - last_print_time > 500) {
      if (my_distance <= 350.0f) {
        Serial.printf("OBSTACLE Detected in %.2f mm with an angle of %.2f deg. Entering State: %d\n", my_distance, my_angle, current_state);
      } else {
        Serial.printf("CLEAR! (Closest object: %.2f mm at %.2f deg)\n", my_distance, my_angle);
      }
      last_print_time = millis();
    }
    if (millis() - my_timestamp > 500)
    {
      current_state = STATE_STOPPED;
    }
    else
    {
      if (my_distance > 350.0f) {
        current_state = STATE_WALKING_FORWARD;
      } else if(my_distance <= 300.0f) {
        current_state = STATE_SKID_TURN;
      } // Hysterisis 
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// --- MAIN SETUP ---
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n--- Starting Quadruped Brain ---");

  LidarSerial.begin(460800, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  obstacleMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(LidarTask, "Lidar Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(WalkingTask, "Walking Gait", 10000, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
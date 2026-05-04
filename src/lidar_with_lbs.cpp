#include <Arduino.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <LDS.h>
#include <LDS_RPLIDAR_A1.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // M_PI

#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

HardwareSerial LidarSerial(2);
LDS_RPLIDAR_A1 lidar;

float shadow_distance = 9999.0f;
float shadow_angle = 0.0f;

// Variables for both tasks

struct ObstacleData
{
  float closest_distance;
  float closest_angle;

  ObstacleData() : closest_distance(9999.0f), closest_angle(0.0f) {} // Our Constructor
};

ObstacleData current_obstacle;
SemaphoreHandle_t obstacleMutex;

int lidar_serial_read_callback()
{
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const  uint8_t * buffer,size_t lenght)
{
  return LidarSerial.write(buffer,lenght);
}


void lidarPointCallback( float angle_deg, float distance_mm, float quality, bool scan_completed)
{
  if (quality == 0 || distance_mm == 0 || distance_mm > 1000.0f) return;
  if (angle_deg > 30.0f && angle_deg < 330.0f) return;

  if(distance_mm < shadow_distance)
  {
    shadow_distance = distance_mm;
    shadow_angle = angle_deg;
  }

  if(scan_completed == true)
  {
    if(xSemaphoreTake(obstacleMutex, portMAX_DELAY) == pdTRUE)
    {
      current_obstacle.closest_distance = shadow_distance;
      current_obstacle.closest_angle = shadow_angle;

      xSemaphoreGive(obstacleMutex);
    }

  shadow_distance = 9999.0f;
  shadow_angle = 0.0f;
  } 
}

void LidarTask(void *pvParameters)
{
  while (true)
  {
    lidar.loop();

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


//ROBOT STATES

enum RobotState
{
  STATE_WALKING_FORWARD,
  STATE_STOPPED,
  STATE_SKID_TURN
};

RobotState current_state = STATE_STOPPED;

void WalkingTask(void *pvParameters)
{

  unsigned long  last_print_time = 0;

  while(true)
  {
    float my_distance = 9999.0f;
    float my_angle = 0.0f;

    if (xSemaphoreTake(obstacleMutex,portMAX_DELAY) == pdTRUE)
    {
      my_distance = current_obstacle.closest_distance;
      my_angle = current_obstacle.closest_angle;

      xSemaphoreGive(obstacleMutex);
    }

    if( millis() - last_print_time > 500)
    {
      if (my_distance <= 350.0f)
      {
        Serial.printf("OBSTACLE Detected in %.2f mm with an angle of %.2f deg. Entering State: %d\n", my_distance, my_angle, current_state);
      }
      else
      {
        Serial.printf("CLEAR! (Closest object in cone: %.2f mm at %.2f deg)\n", my_distance, my_angle);
      }

      last_print_time = millis();

    }

    if (my_distance > 350.0f)
    {
      current_state = STATE_WALKING_FORWARD;
    }
    else if(my_distance <= 300.0f)
    {
      current_state = STATE_SKID_TURN;
    }

    switch (current_state)
    {
      case STATE_WALKING_FORWARD:

        break;

      case STATE_STOPPED:

        break;

      case STATE_SKID_TURN:

        break;
    }

    vTaskDelay (pdMS_TO_TICKS(10)); //Sleep for 10 ms
    //vTaskDelay(10 / portTICK_PERIOD_MS); //is the same

  }
}


void setup() {

  Serial.begin(115200);
  Serial.println("Starting Quadruped Brain");

  LidarSerial.begin(460800, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setScanPointCallback(lidarPointCallback);
  lidar.init();
  lidar.start();

  obstacleMutex = xSemaphoreCreateMutex();


  xTaskCreatePinnedToCore(
    LidarTask, // Function name
    "Lidar Task", // Personal Name
    10000, // Stack size
    NULL, // Task Params
    1, //Priority
    NULL, // Task Handle
    0   // Pin to Core  0
  );

  xTaskCreatePinnedToCore(
    WalkingTask,
    "Walking Gait",
    10000,
    NULL,
    1,
    NULL,
    1
  );

}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelete(NULL);
}

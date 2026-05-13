#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>


#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

#define SDA_PIN 23
#define SCL_PIN 22

// opws ta blepw apo mprosta
// FL_HIP, FL_KNEE, FR_HIP, FR_KNEE, BL_HIP, BL_KNEE, BR_HIP, BR_KNEE 
const uint8_t servo_channels[8] = {4, 7, 2, 5, 0, 6, 3, 1};

// Same order as above
// hip : sta aristera otan prostheteis paei to hip pros ta katw, enw sta dejia pros ta panw
// knee: sta aristera otan prostheteis paei to horn pros ta <- (mprosta), enw sta dejia pros ta -> (dld to katw podi paei antitheta)
const uint16_t servo_homes[8] = {312, 307, 312, 307, 318, 307, 307, 307};
//const uint16_t servo_homes[8] = {315, 315, 307, 307, 300, 285, 330, 280}; // middle values
//const uint16_t servo_homes[8] = {365, 303, 260, 335, 345, 270, 280, 285};// one standing position
const uint16_t servo_standing[8] = {430, 370, 192, 235, 440, 370, 166, 235};

const int8_t servo_dirs[8] = {-1, -1, 1, 1, -1, -1, 1, 1}; //inversion

const float hip_home_angles[4] = {85.0f, 85.0f, 80.0f, 90.0f}; // angle between the hip and the servo 
const float knee_home_angles[4] = {90.0f, 90.0f, 90.0f, 90.0f}; // angle between the hip and the lower leg

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

HardwareSerial LidarSerial(2);


float shadow_distance = 9999.0f;
float shadow_angle = 0.0f;
const float TICKS_PER_DEGREE = 2.27f;

volatile bool lidar_is_ready = false;

// Variables for both tasks

struct ObstacleData
{
  float closest_distance;
  float closest_angle;
  unsigned long timestamp;

  ObstacleData() : closest_distance(9999.0f), closest_angle(0.0f) {} // Our Constructor
};

ObstacleData current_obstacle;
SemaphoreHandle_t obstacleMutex;

void standStill() {
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(servo_channels[i], 0, servo_standing[i]);
  }
}

void home_pose() {
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(servo_channels[i], 0, servo_homes[i]);
  }
}

int getPWM(float angleOffset, uint16_t homeValue)
{
  int pulse = homeValue + (angleOffset * TICKS_PER_DEGREE);
  pulse = constrain(pulse,102,512); // 102 min kai 512 max (1 tick is almost 4.88μs, so 500 μs /4.88 μs/ticks = 102 , 2500/2.88 = 512 ticks )

  return pulse;
}


void InverseKinematics(int leg_num, float target_x, float target_z)
{
  float d_squared = (target_x * target_x) + (target_z * target_z);
  float distance = sqrt(d_squared);
  float L1 = 100.0f;
  //float L1 = 140.0f;
  float L2 = 114.0f;
  //float L2 = 150.0f;

  if (distance > (L1 + L2))
  {
    Serial.println("Out of Reach!");
    return;
  }

  // IK

  float numerator = (L1 * L1) + (L2 * L2) - d_squared;
  float denominator = 2.0f * L1 * L2;
  float cos_angle = numerator / denominator;
  cos_angle = constrain(cos_angle, -1.0f, 1.0f);
  float target_knee_angle = acos(cos_angle) * (180.0f / M_PI);

  float alpha_rad = atan2 (target_x, -target_z );
  float beta_arg = (((L1 * L1) + d_squared - (L2 * L2)) / (2.0f * L1 * distance));
  beta_arg = constrain(beta_arg,-1.0f,1.0);
  float beta_rad = acos(beta_arg);
  float target_hip_angle = (alpha_rad + beta_rad) * (180.0f / M_PI);

  int hip_idx = leg_num * 2;
  int knee_idx = (leg_num * 2)+1;

  float hip_offset = target_hip_angle - hip_home_angles[leg_num];
  float knee_offset = target_knee_angle - knee_home_angles[leg_num];

  hip_offset = hip_offset * servo_dirs[hip_idx];
  knee_offset = knee_offset * servo_dirs[knee_idx];

  pwm.setPWM(servo_channels[hip_idx], 0 , getPWM(hip_offset, servo_homes[hip_idx]));
  pwm.setPWM(servo_channels[knee_idx],0,getPWM(knee_offset,servo_homes[knee_idx]));

  Serial.printf("Target: (%.1f, %.1f) -> Hip: %.1f°, Knee: %.1f°\n", target_x, target_z, target_hip_angle, target_knee_angle);
  Serial.printf("Leg %d: Hip PWM=%d, Knee PWM=%d (offsets: %.1f°, %.1f°)\n", leg_num, getPWM(hip_offset, servo_homes[hip_idx]), getPWM(knee_offset,servo_homes[knee_idx]), hip_offset, knee_offset);
}

struct Point2D
{
  float x;
  float z;
};

const Point2D step_trajectory[10] = 
{
  /*{0.0f , -100.0f}, // 1. Lift : Pull the foot up
  {40.0f, -80.0f}, // 2. Reach : Move the foot forward in the air
  {40.0f, -120.0f}, // 3. Plant: Put the foot down on the ground
  {0.0f, -150.0f} // 4. Pull : Ddrage the foot backwards (So the body moves foward)*/

  // --- SWING PHASE ---

  {-20.0f, -150.0f}, // 1. Lift off the ground
  {-10.0f, -135.0f}, // 2. Swing forward and up
  {  0.0f, -130.0f}, // 3. Peak height (clearing obstacles)
  { 10.0f, -135.0f}, // 4. Swing forward and start lowering
  { 20.0f, -150.0f}, // 5. Nearing the ground

  // --- STANCE PHASE ---

  { 20.0f, -170.0f}, // 6. Plant foot firmly on the ground
  { 10.0f, -170.0f}, // 7. Pulling body forward
  {  0.0f, -170.0f}, // 8. Mid-stance (leg straight down)
  {-10.0f, -170.0f}, // 9. Pushing body forward
  {-20.0f, -170.0f}  // 10. Max extension, ready to lift again


};

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
      if (packet_idx == 0) 
      {
        uint8_t sync = b & 0x03;
        if (sync == 0x01 || sync == 0x02) 
        { 
          packet[0] = b;
          packet_idx = 1;
        }
      }

      // BYTE 1: Angle Checkbit
      else if (packet_idx == 1) 
      {
        if ((b & 0x01) == 0x01) 
        { // Checkbit must be 1
          packet[1] = b;
          packet_idx = 2;
        } else 
        {
          packet_idx = 0; // Invalid, reset
          
          // Re-evaluate current byte just in case it's actually Byte 0
          uint8_t sync = b & 0x03;
          if (sync == 0x01 || sync == 0x02) 
          {
            packet[0] = b;
            packet_idx = 1;
          }
        }
      }
      // BYTES 2, 3, 4: Data
      else 
      {
        packet[packet_idx] = b;
        packet_idx++;

        if (packet_idx == 5) 
        {
          packet_idx = 0; // Reset for the next packet

          // math for data transmition
          bool startBit = (packet[0] & 0x01) == 1;
          byte quality = packet[0] >> 2;
          float angle_deg = ((packet[2] << 7) | (packet[1] >> 1)) / 64.0f;
          float distance_mm = ((packet[4] << 8) | packet[3]) / 4.0f;

          if(startBit) 
          {
            if(xSemaphoreTake(obstacleMutex, portMAX_DELAY) == pdTRUE) 
            {
              current_obstacle.closest_distance = shadow_distance;
              current_obstacle.closest_angle = shadow_angle;
              current_obstacle.timestamp = millis();
              xSemaphoreGive(obstacleMutex);

              lidar_is_ready = true;
            }
            shadow_distance = 9999.0f;
            shadow_angle = 0.0f;
          } 

          // Objects only in fornt
          if (quality > 0 && distance_mm > 0 && distance_mm <= 1000.0f) 
          {
            if (angle_deg <= 30.0f || angle_deg >= 330.0f) 
            {
              if(distance_mm < shadow_distance) 
              {
                shadow_distance = distance_mm;
                shadow_angle = angle_deg;
              }
            }
          }
        }
      }
    }

    // Keep the motor alive if the Lidar gets stuck
    if (got_data) 
    {
      last_data_time = millis();
    } 
    else 
    {
      if (millis() - last_data_time > 1500) 
      {
        LidarSerial.write(startMotorCmd, sizeof(startMotorCmd));
        last_data_time = millis();
      }
    }

    // for the Watchdog
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}


//ROBOT STATES

enum RobotState
{
  STATE_WALKING_FORWARD,
  STATE_STOPPED,
  STATE_SKID_TURN,
  STATE_BOOTING,
  STATE_TEST_STEP,
  STATE_RESUME
};

RobotState current_state = STATE_BOOTING;
RobotState previous_state = STATE_BOOTING;

void WalkingTask(void *pvParameters)
{

  unsigned long  last_print_time = 0;

  int total_points = sizeof(step_trajectory) / sizeof(step_trajectory[0]);
  int steps = 15;

  static int traj_index = 0;  // which point of the trajectory we are on
  static int interp_step = 1; // which point of the microsteps in interpolation we are on

  // variables for tracking real time position fo the legs
  static float current_x1 = 0.0f, current_z1 = -150.0f;
  static float current_x2 = 0.0f, current_z2 = -150.0f;

  static int stop_step = 1;
  static float end_point_x1, end_point_z1, end_point_x2, end_point_z2;

  while(true)
  {
    float my_distance = 9999.0f;
    float my_angle = 0.0f;
    unsigned long my_timestamp = 0;

    if (xSemaphoreTake(obstacleMutex,portMAX_DELAY) == pdTRUE)
    {
      my_distance = current_obstacle.closest_distance;
      my_angle = current_obstacle.closest_angle;
      my_timestamp = current_obstacle.timestamp;

      xSemaphoreGive(obstacleMutex);
    }

    if( millis() - last_print_time > 1500)
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

    if (millis() - my_timestamp > 1500)
    {
      current_state = STATE_STOPPED;
    }
    else if(current_state != STATE_BOOTING && current_state != STATE_TEST_STEP)
    {
      if (my_distance > 350.0f)
      {
        if(current_state == STATE_STOPPED && stop_step > steps)
        {
          current_state = STATE_RESUME;
          //current_state = STATE_TEST_STEP;
        }
        else if (current_state == STATE_SKID_TURN)
        {
          current_state = STATE_STOPPED;
        }
      } 
      else if(my_distance <= 300.0f && (current_state == STATE_WALKING_FORWARD || current_state == STATE_RESUME)) 
      {
        current_state = STATE_STOPPED;
      } // Hysterisis 
      else if(my_distance <= 300.0f && current_state == STATE_STOPPED && stop_step > steps) 
      {
        current_state = STATE_SKID_TURN;
      } 
    }

    switch (current_state)
    {
      case STATE_BOOTING:
      if(current_state != previous_state)
      {
        standStill();
      }

      while(!lidar_is_ready)
      {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
        vTaskDelay(pdMS_TO_TICKS(2000));
        //current_state = STATE_STOPPED;
        current_state = STATE_WALKING_FORWARD;
        break;

      case STATE_WALKING_FORWARD: 
      {
          // PAIR 1
          Point2D current_point_1 = step_trajectory[traj_index];
          Point2D next_point_1 = step_trajectory[(traj_index + 1) % total_points];

          // PAIR 2
          int shifted_i = (traj_index + 5) % total_points;
          Point2D current_point_2 = step_trajectory [shifted_i];
          Point2D next_point_2 = step_trajectory[(shifted_i + 1) % total_points];

            float t = (float) interp_step / steps;// with linear interpolation
            //float t = (1.0f - cos((float)interp_step / steps * M_PI)) / 2.0f; //cosine interpolation

            current_x1 = current_point_1.x + ((next_point_1.x - current_point_1.x) * t);
            current_z1 = current_point_1.z + ((next_point_1.z - current_point_1.z) * t);

            current_x2 = current_point_2.x + ((next_point_2.x - current_point_2.x) * t);
            current_z2 = current_point_2.z + ((next_point_2.z - current_point_2.z) * t);

            InverseKinematics(0, current_x1, current_z1);
            InverseKinematics(3, current_x1, current_z1);

            InverseKinematics(1, current_x2, current_z2);
            InverseKinematics(2, current_x2, current_z2);

            interp_step++;
            if(interp_step > steps)
            {
              interp_step = 1;
              traj_index = (traj_index + 1) % total_points;
            }

            stop_step = 1;
            vTaskDelay(pdMS_TO_TICKS(15));
            //home_pose();
            break;
      }

      case STATE_RESUME:
      {
        static int resume_step = 1;
        float standing_z = -150.0f;

        Point2D target1 = step_trajectory[traj_index];
        Point2D target2 = step_trajectory[(traj_index + 5) % total_points];

        if (resume_step <= steps)
        {
          float t = (float) resume_step / steps;

          current_x1 = 0.0f + ((target1.x - 0.0f) * t);
          current_z1 = standing_z + ((target1.z - standing_z) * t);
          
          current_x2 = 0.0f + ((target2.x - 0.0f) * t);
          current_z2 = standing_z + ((target2.z - standing_z) * t);
          
          InverseKinematics(0, current_x1, current_z1);
          InverseKinematics(3, current_x1, current_z1);
          InverseKinematics(1, current_x2, current_z2);
          InverseKinematics(2, current_x2, current_z2);
          
          resume_step++;
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        else
        {
          resume_step = 1;
          current_state = STATE_WALKING_FORWARD;
        }
      }

      case STATE_STOPPED:
      {
        if(current_state != previous_state)
        {
          end_point_x1 = current_x1;
          end_point_z1 = current_z1;
          end_point_x2 = current_x2;
          end_point_z2 = current_z2;
        }
          if(stop_step <= steps)
          {
            float t = (float) stop_step / steps;
            float standing_z = -150.0f;

            current_x1 = end_point_x1 + ((0.0f - end_point_x1) * t);
            current_z1 = end_point_z1 + ((standing_z - end_point_z1) * t);

            current_x2 = end_point_x2 + ((0.0f - end_point_x2) * t);
            current_z2 = end_point_z2 + ((standing_z - end_point_z2) * t);  

            InverseKinematics(0, current_x1, current_z1);
            InverseKinematics(3, current_x1, current_z1);
            InverseKinematics(1, current_x2, current_z2);
            InverseKinematics(2, current_x2, current_z2);
            
            stop_step++;
            vTaskDelay(pdMS_TO_TICKS(20));
          }
          else
          {
            vTaskDelay(pdMS_TO_TICKS(50));
          }
         //home_pose();
        break;
      }

      case STATE_SKID_TURN:
      {
        // PAIR 1
        Point2D current_point_1 = step_trajectory[traj_index];
        Point2D next_point_1 = step_trajectory[(traj_index + 1) % total_points];

        // PAIR 2
        int shifted_i = (traj_index + 5) % total_points;
        Point2D current_point_2 = step_trajectory [shifted_i];
        Point2D next_point_2 = step_trajectory[(shifted_i + 1) % total_points];

        float t = (float) interp_step / steps;// with linear interpolation
        //float t = (1.0f - cos((float)interp_step / steps * M_PI)) / 2.0f; //cosine interpolation

        current_x1 = current_point_1.x + ((next_point_1.x - current_point_1.x) * t);
        current_z1 = current_point_1.z + ((next_point_1.z - current_point_1.z) * t);

        current_x2 = current_point_2.x + ((next_point_2.x - current_point_2.x) * t);
        current_z2 = current_point_2.z + ((next_point_2.z - current_point_2.z) * t);

        InverseKinematics(0, current_x1, current_z1);
        InverseKinematics(3, -1*current_x1, current_z1);

        InverseKinematics(1, -1*current_x2, current_z2);
        InverseKinematics(2, current_x2, current_z2);

        interp_step++;
        if(interp_step > steps)
        {
          interp_step = 1;
          traj_index = (traj_index + 1) % total_points;
        }

        stop_step = 1;
        vTaskDelay(pdMS_TO_TICKS(15));
        break;
      }

      case STATE_TEST_STEP:
        /*for (int i = 0; i < (sizeof(step_trajectory) / sizeof(step_trajectory[0])); i++)
        {
          //InverseKinematics(0, step_trajectory[i].x, step_trajectory[i].z);
          //InverseKinematics(0,  20.0f, -100.0f);
          InverseKinematics(3, step_trajectory[i].x, step_trajectory[i].z);
          //InverseKinematics(1, step_trajectory[i].x, step_trajectory[i].z);
          //InverseKinematics(2, step_trajectory[i].x, step_trajectory[i].z);
          //InverseKinematics(3, step_trajectory[i].x, step_trajectory[i].z);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        //current_state = STATE_STOPPED;
        InverseKinematics(1, 0.0f, -100.0f);
        vTaskDelay(pdMS_TO_TICKS(500));
        InverseKinematics(1, 0.0f, -150.0f);
        vTaskDelay(pdMS_TO_TICKS(500));*/

        //standStill();
        break;
    }

    previous_state = current_state;

    //vTaskDelay (pdMS_TO_TICKS(10)); //Sleep for 10 ms
    //vTaskDelay(10 / portTICK_PERIOD_MS); //is the same

  }
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println(" \n --- Starting Quadruped Brain ---");

  LidarSerial.begin(460800, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  obstacleMutex = xSemaphoreCreateMutex();

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50);

  standStill();

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
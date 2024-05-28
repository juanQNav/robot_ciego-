/*
 * File:          right-hand_rule.c
 * Date: 24/05/2024
 * Description: Robot Kehepera IV.
 *              This robot have 5 ultrasonic sensors. We only use 2 ultrasonic sensors. To avoid collisions, the robot must use these proximity sensors. To exit the maze, the robot must use the right hand rule.
 * Author: QUISTIAN NAVARRO JUAN LUIS
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/inertial_unit.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TIME_STEP 32
#define MAX_SPEED 10
#define TURN_SPEED_FACTOR 0.3
#define DISTANCE_THRESHOLD 0.29

// Define matrices and variables for the Kalman Filter of the front sensor
float x_front_k[2] = {0, 0};                       // state vector: [distance, velocity].
float P_front_k[2][2] = {{1, 0}, {0, 1}};          // Covariance matrix
float F[2][2] = {{1, TIME_STEP / 1000.0}, {0, 1}}; // State transition matrix
float Q[2][2] = {{1e-4, 0}, {0, 1e-4}};            // Covariance matrix of process noise
float H[2] = {1, 0};                               // Matriz de transición de estados
float R = 0.05;                                    // Covariance matrix of the measurement noise
float I[2][2] = {{1, 0}, {0, 1}};                  // Identity matrix

// Define matrices and variables for the Kalman Filter of the left sensor
float x_right_k[2] = {0, 0};              // state vector: [distance, velocity].
float P_right_k[2][2] = {{1, 0}, {0, 1}}; // Covariance matrix

void kalman_predict(float x_k[], float P_k[][2])
{
  // Estimation of the predicted state
  float x_k_1[2];
  x_k_1[0] = F[0][0] * x_k[0] + F[0][1] * x_k[1];
  x_k_1[1] = F[1][0] * x_k[0] + F[1][1] * x_k[1];

  // Estimation of predicted covariance
  float P_k_1[2][2];
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P_k_1[i][j] = F[i][0] * P_k[0][j] + F[i][1] * P_k[1][j] + Q[i][j];
    }
  }

  // Update status and covariance
  x_k[0] = x_k_1[0];
  x_k[1] = x_k_1[1];
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P_k[i][j] = P_k_1[i][j];
    }
  }
}

void kalman_update(float x_k[], float P_k[][2], float z)
{
  // Innovation or measurement residual
  float y_k = z - (H[0] * x_k[0] + H[1] * x_k[1]);

  // Covariance of innovation
  float S_k = H[0] * P_k[0][0] * H[0] + H[0] * P_k[0][1] * H[1] + H[1] * P_k[1][0] * H[0] + H[1] * P_k[1][1] * H[1] + R;

  // Kalman gain
  float K_k[2];
  K_k[0] = (P_k[0][0] * H[0] + P_k[0][1] * H[1]) / S_k;
  K_k[1] = (P_k[1][0] * H[0] + P_k[1][1] * H[1]) / S_k;

  // Estimated updated status
  x_k[0] = x_k[0] + K_k[0] * y_k;
  x_k[1] = x_k[1] + K_k[1] * y_k;

  // Updated covariance estimation
  float P_k_1[2][2];
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P_k_1[i][j] = (I[i][j] - K_k[i] * H[j]) * P_k[i][j];
    }
  }
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P_k[i][j] = P_k_1[i][j];
    }
  }
}

// Function to turn the robot by 90 degrees
void turn(int dir, WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag iu)
{
  // Get current yaw
  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(iu);
  double c_yaw = rpy[2];
  c_yaw = round(c_yaw / (M_PI / 2)) * M_PI / 2;
  double target_yaw = fmod((c_yaw + (M_PI / 2) * dir), (2 * M_PI));

  if (target_yaw > M_PI)
  {
    target_yaw -= 2 * M_PI;
  }
  if (target_yaw < -M_PI)
  {
    target_yaw += 2 * M_PI;
  }

  printf("Current yaw: %f, Target yaw: %f\n", c_yaw, target_yaw);

  double turn_speed = MAX_SPEED * TURN_SPEED_FACTOR;
  wb_motor_set_velocity(left_motor, -turn_speed * dir);
  wb_motor_set_velocity(right_motor, turn_speed * dir);

  while (wb_robot_step(TIME_STEP) != -1)
  {
    rpy = wb_inertial_unit_get_roll_pitch_yaw(iu);
    if (fabs(rpy[2] - target_yaw) < 0.03)
    {
      break;
    }
  }

  // Continue forward
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

// Function to move the robot forward
void forward(WbDeviceTag left_motor, WbDeviceTag right_motor)
{
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

// Function to turn the robot by 270 degrees to the left (equivalent to three 90-degree right turns)
void turn_270_left(WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag iu)
{
  for (int i = 0; i < 3; i++)
  {
    turn(-1, left_motor, right_motor, iu); // Turn 90 degrees to the right
  }
}

int main(int argc, char **argv)
{
  // Initializing the Webots environment
  wb_robot_init();

  // Set the step time for the simulation
  int time_step = (int)wb_robot_get_basic_time_step();

  // Enable and obtain the front ultrasonic sensor and set its passing time.
  WbDeviceTag front_ultrasonic_sensor = wb_robot_get_device("front ultrasonic sensor");
  wb_distance_sensor_enable(front_ultrasonic_sensor, time_step);
  WbDeviceTag right_ultrasonic_sensor = wb_robot_get_device("right ultrasonic sensor");
  wb_distance_sensor_enable(right_ultrasonic_sensor, time_step);

  // Obtain and configure the left and right motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0); // Set initial velocity to 0
  wb_motor_set_velocity(right_motor, 0.0);

  // Inertial unit
  WbDeviceTag iu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(iu, time_step);

  bool flag = true;
  bool flag_after_right = false;

  char str_message[150];

  while (wb_robot_step(time_step) != -1)
  {
    // Read distance from ultrasonic sensor
    float front_distance = wb_distance_sensor_get_value(front_ultrasonic_sensor);
    float right_distance = wb_distance_sensor_get_value(right_ultrasonic_sensor);

    // Apply the Kalman filter for the front sensor
    kalman_predict(x_front_k, P_front_k);
    kalman_update(x_front_k, P_front_k, front_distance);
    float front_distance_kalman = x_front_k[0];

    // Apply the Kalman filter for the left sensor
    kalman_predict(x_right_k, P_right_k);
    kalman_update(x_right_k, P_right_k, right_distance);
    float right_distance_kalman = x_right_k[0];

    // Deciding the robot's movement
    bool there_front_wall = front_distance_kalman <= DISTANCE_THRESHOLD;
    bool there_right_wall = right_distance_kalman <= DISTANCE_THRESHOLD;
    bool no_walls = !there_front_wall && !there_right_wall;

    if (no_walls && flag)
    { // The robot has just started
      strcpy(str_message, "No walls");
      forward(left_motor, right_motor);
    }
    else if (there_front_wall)
    { // Turn right
      strcpy(str_message, "There is a wall in front");
      turn(1, left_motor, right_motor, iu);   // Turn left (dir=1)
      wb_motor_set_velocity(left_motor, 0.0); // Set initial velocity to 0
      wb_motor_set_velocity(right_motor, 0.0);
      wb_robot_step(TIME_STEP * 18);
      flag = false;
      flag_after_right = false;
    }
    else if (there_right_wall)
    {
      strcpy(str_message, "There is a wall on the right");
      forward(left_motor, right_motor);
      flag_after_right = true;
    }
    else if (no_walls && flag_after_right)
    {
      strcpy(str_message, "No walls, turn 90° to right");
      forward(left_motor, right_motor);
      wb_robot_step(TIME_STEP * 18);
      turn(-1, left_motor, right_motor, iu); // Turn right (dir=-1)
      forward(left_motor, right_motor);
      wb_robot_step(TIME_STEP * 30);
      flag_after_right = false;
      wb_motor_set_velocity(left_motor, 0.0); // Set initial velocity to 0
      wb_motor_set_velocity(right_motor, 0.0);
      wb_robot_step(TIME_STEP * 18);
    }
    // Print information for debugging
    printf("message: %s\n", str_message);
    printf("front distance (Kalman): %f\n", front_distance_kalman);
    printf("front distance (sensor): %f\n", front_distance);
    printf("right distance (Kalman): %f\n", right_distance_kalman);
    printf("right distance (sensor): %f\n", right_distance);
    printf("-----------------------\n");
  };

  // Clean before leaving
  wb_robot_cleanup();

  return 0;
}

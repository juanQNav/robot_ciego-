// Quistian Navarro Juan Luis
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/camera_recognition_object.h>

#define TIME_STEP 64

int main()
{
  // Initialize the Webots environment
  wb_robot_init();

  // Set the time step for the simulation
  int time_step = (int)wb_robot_get_basic_time_step();

  // Enable the camera and set its time step
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  // Enable object recognition for the camera and set its time step
  wb_camera_recognition_enable(camera, TIME_STEP);

  // Enable and get the front ultrasonic sensor and set its time step
  WbDeviceTag ultrasonic_sensor = wb_robot_get_device("front ultrasonic sensor");
  wb_distance_sensor_enable(ultrasonic_sensor, time_step);

  // Get and set the left and right motors
  WbDeviceTag left_motor, right_motor;
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY); // Set motor position to infinity for speed control
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0); // Set initial velocity to 0
  wb_motor_set_velocity(right_motor, 0.0);

  int last_display_second = 0;

  // Main control loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    int display_second = (int)wb_robot_get_time();
    if (display_second != last_display_second)
    {
      last_display_second = display_second;

      // Read distance from ultrasonic sensor
      float distance = wb_distance_sensor_get_value(ultrasonic_sensor);

      // Get number of recognized objects
      int num_ob = wb_camera_recognition_get_number_of_objects(camera);
      printf("number of objects: %d\n", num_ob);
      if (num_ob > 0)
      {
        // Get information about recognized objects
        const WbCameraRecognitionObject *ob = wb_camera_recognition_get_objects(camera);

        // Find the largest object
        const WbCameraRecognitionObject *largest_object = &ob[0];
        for (int i = 1; i < num_ob; i++)
        {
          if (ob[i].size[0] * ob[i].size[1] > largest_object->size[0] * largest_object->size[1])
          {
            largest_object = &ob[i];
          }
        }

        // Handle robot movement based on the detected object and distance
        if (largest_object->colors[1] && distance < 0.29)
        {
          printf("**** left detected green!!!!\n");
          wb_motor_set_velocity(left_motor, 0);
          wb_motor_set_velocity(right_motor, 8);
        }
        else if (largest_object->colors[0] && distance < 0.29)
        {
          printf("**** right detected red!!!!\n");
          wb_motor_set_velocity(left_motor, 8);
          wb_motor_set_velocity(right_motor, 0);
        }
        else
        {
          // Move forward if no obstacles are detected
          wb_motor_set_velocity(left_motor, 10);
          wb_motor_set_velocity(right_motor, 10);
        }

        // Print information for debugging
        printf("red: %f \n", largest_object->colors[0]);
        printf("green: %f \n", largest_object->colors[1]);
        printf("blue: %f \n", largest_object->colors[2]);
        printf("distance: %f\n", distance);
        printf("-----------------------\n");
      }
    }
  }

  // Cleanup before exiting
  wb_robot_cleanup();

  return 0;
}

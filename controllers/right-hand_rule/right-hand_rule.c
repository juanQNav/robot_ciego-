/*
 * Archivo:          right-hand_rule.c
 * Fecha: 24/05/2024
 * Descripción: Robot Kehepera IV.
 *              Este robot tiene 5 sensores ultrasónicos. Solo usamos 2 sensores ultrasónicos. Para evitar colisiones, el robot debe usar estos sensores de proximidad. Para salir del laberinto, el robot debe usar la regla de la mano derecha.
 * Autor: QUISTIAN NAVARRO JUAN LUIS
 * Modificaciones:
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

// Definir matrices y variables para el filtro de Kalman del sensor frontal
float x_front_k[2] = {0, 0};                       // vector de estado: [distancia, velocidad].
float P_front_k[2][2] = {{1, 0}, {0, 1}};          // Matriz de covarianza
float F[2][2] = {{1, TIME_STEP / 1000.0}, {0, 1}}; // Matriz de transición de estado
float Q[2][2] = {{1e-4, 0}, {0, 1e-4}};            // Matriz de covarianza del ruido del proceso
float H[2] = {1, 0};                               // Matriz de observación
float R = 0.05;                                    // Matriz de covarianza del ruido de medición
float I[2][2] = {{1, 0}, {0, 1}};                  // Matriz identidad

// Definir matrices y variables para el filtro de Kalman del sensor DERECHO
float x_right_k[2] = {0, 0};              // vector de estado: [distancia, velocidad].
float P_right_k[2][2] = {{1, 0}, {0, 1}}; // Matriz de covarianza

void kalman_predict(float x_k[], float P_k[][2])
{
  // Estimación del estado predicho
  float x_k_1[2];
  x_k_1[0] = F[0][0] * x_k[0] + F[0][1] * x_k[1];
  x_k_1[1] = F[1][0] * x_k[0] + F[1][1] * x_k[1];

  // Estimación de la covarianza predicha
  float P_k_1[2][2];
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P_k_1[i][j] = F[i][0] * P_k[0][j] + F[i][1] * P_k[1][j] + Q[i][j];
    }
  }

  // Actualizar estado y covarianza
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
  // Innovación o residual de medición
  float y_k = z - (H[0] * x_k[0] + H[1] * x_k[1]);

  // Covarianza de la innovación
  float S_k = H[0] * P_k[0][0] * H[0] + H[0] * P_k[0][1] * H[1] + H[1] * P_k[1][0] * H[0] + H[1] * P_k[1][1] * H[1] + R;

  // Ganancia de Kalman
  float K_k[2];
  K_k[0] = (P_k[0][0] * H[0] + P_k[0][1] * H[1]) / S_k;
  K_k[1] = (P_k[1][0] * H[0] + P_k[1][1] * H[1]) / S_k;

  // Estado actualizado estimado
  x_k[0] = x_k[0] + K_k[0] * y_k;
  x_k[1] = x_k[1] + K_k[1] * y_k;

  // Estimación de la covarianza actualizada
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

// Función para girar el robot 90 grados
void turn(int dir, WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag iu)
{
  // Obtener el yaw actual
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

  printf("Yaw actual: %f, Yaw objetivo: %f\n", c_yaw, target_yaw);

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

  // Continuar hacia adelante
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

// Función para mover el robot hacia adelante
void forward(WbDeviceTag left_motor, WbDeviceTag right_motor)
{
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

int main(int argc, char **argv)
{
  // Inicializando el entorno Webots
  wb_robot_init();

  // Establecer el tiempo de paso para la simulación
  int time_step = (int)wb_robot_get_basic_time_step();

  // Habilitar y obtener el sensor ultrasónico frontal y configurar su tiempo de paso.
  WbDeviceTag front_ultrasonic_sensor = wb_robot_get_device("front ultrasonic sensor");
  wb_distance_sensor_enable(front_ultrasonic_sensor, time_step);
  WbDeviceTag right_ultrasonic_sensor = wb_robot_get_device("right ultrasonic sensor");
  wb_distance_sensor_enable(right_ultrasonic_sensor, time_step);

  // Obtener y configurar los motores izquierdo y derecho
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0); // Establecer velocidad inicial a 0
  wb_motor_set_velocity(right_motor, 0.0);

  // Unidad inercial
  WbDeviceTag iu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(iu, time_step);

  bool flag = true;
  bool flag_after_right = false;

  char str_message[150];

  while (wb_robot_step(time_step) != -1)
  {
    // Leer distancia del sensor ultrasónico
    float front_distance = wb_distance_sensor_get_value(front_ultrasonic_sensor);
    float right_distance = wb_distance_sensor_get_value(right_ultrasonic_sensor);

    // Aplicar el filtro de Kalman para el sensor frontal
    kalman_predict(x_front_k, P_front_k);
    kalman_update(x_front_k, P_front_k, front_distance);
    float front_distance_kalman = x_front_k[0];

    // Aplicar el filtro de Kalman para el sensor derecho
    kalman_predict

        (x_right_k, P_right_k);
    kalman_update(x_right_k, P_right_k, right_distance);
    float right_distance_kalman = x_right_k[0];

    // Decidir el movimiento del robot
    bool there_front_wall = front_distance_kalman <= DISTANCE_THRESHOLD;
    bool there_right_wall = right_distance_kalman <= DISTANCE_THRESHOLD;
    bool no_walls = !there_front_wall && !there_right_wall;

    if (no_walls && flag)
    { // El robot acaba de empezar
      strcpy(str_message, "Sin paredes");
      forward(left_motor, right_motor);
    }
    else if (there_front_wall)
    { // Girar a la derecha
      strcpy(str_message, "Hay una pared enfrente");
      turn(1, left_motor, right_motor, iu);   // Girar a la izquierda (dir=1)
      wb_motor_set_velocity(left_motor, 0.0); // Establecer velocidad inicial a 0
      wb_motor_set_velocity(right_motor, 0.0);
      wb_robot_step(TIME_STEP * 18);
      flag = false;
      flag_after_right = false;
    }
    else if (there_right_wall)
    {
      strcpy(str_message, "Hay una pared a la derecha");
      forward(left_motor, right_motor);
      flag_after_right = true;
    }
    else if (no_walls && flag_after_right)
    {
      strcpy(str_message, "Sin paredes, girar 90° a la derecha");
      forward(left_motor, right_motor);
      wb_robot_step(TIME_STEP * 18);
      turn(-1, left_motor, right_motor, iu); // Girar a la derecha (dir=-1)
      forward(left_motor, right_motor);
      wb_robot_step(TIME_STEP * 30);
      flag_after_right = false;
      wb_motor_set_velocity(left_motor, 0.0); // Establecer velocidad inicial a 0
      wb_motor_set_velocity(right_motor, 0.0);
      wb_robot_step(TIME_STEP * 18);
    }
    // Imprimir información para depuración
    printf("mensaje: %s\n", str_message);
    printf("distancia frontal (Kalman): %f\n", front_distance_kalman);
    printf("distancia frontal (sensor): %f\n", front_distance);
    printf("distancia derecha (Kalman): %f\n", right_distance_kalman);
    printf("distancia derecha (sensor): %f\n", right_distance);
    printf("-----------------------\n");
  };

  // Limpiar antes de salir
  wb_robot_cleanup();

  return 0;
}
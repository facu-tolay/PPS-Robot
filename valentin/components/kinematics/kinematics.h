/*
 * Kinematics.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#ifndef COMPONENTS_KINEMATICS_KINEMATICS_H_
#define COMPONENTS_KINEMATICS_KINEMATICS_H_

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "../../main/constants.h"

#pragma pack(push, 1) // FIXME wtf is this?
typedef struct {
    float setpoint;
    float velocidad_lineal_x;
    float velocidad_lineal_y;
    float velocidad_angular;
} movement_vector_t;
#pragma pack(pop)

#define VELOCITY_VECTOR_SIZE    3

/*
 * For sending new setpoints to motor tasks
 * */
typedef struct {
    float setpoint;
    float rpm;
} motor_movement_vector_t;

void calculo_matriz_cinematica_inversa(float *vector_velocidad_lineal, float *vector_velocidad_angular);
void calculo_matriz_cinematica_directa(float *vector_velocidad_angular, float *vector_velocidad_lineal);
void calculo_error_velocidades_lineales(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal);
void calculo_distancia_recorrida_acumulada(float *velocidad_lineal_real, float delta_t, float *distancia_accum, float *delta_distance);
void calculo_compensacion_linea_magnetica(uint8_t is_velocidad_rotacional_zero, float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE], int line_follower_count[HALL_SENSOR_COUNT]);
void calculo_rompensacion_rotacional(float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE]);
void reset_accum();
uint8_t robot_in_radius_of_setpoint(float desired_setpoint, float *current_position);
void seteo_parametros_vectores(float *vector_velocidad_lineal, float *vector_velocidad_angular, movement_vector_t *movement_vector);
void seteo_datos_motor_task(float velocidad_angular, float setpoint, motor_movement_vector_t *motor, QueueHandle_t queue);

#endif /* COMPONENTS_KINEMATICS_KINEMATICS_H_ */

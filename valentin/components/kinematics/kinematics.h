/*
 * Kinematics.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#ifndef COMPONENTS_KINEMATICS_KINEMATICS_H_
#define COMPONENTS_KINEMATICS_KINEMATICS_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/queue.h"
// #include "../../main/main.h"

#pragma pack(push, 1)
typedef struct {
	float setpoint;
	float velocidad_lineal_x;
	float velocidad_lineal_y;
	float velocidad_angular;
} motor_mqtt_params_t;
#pragma pack(pop)

/*
 * For sending new setpoints to motor tasks
 * */
typedef struct {
	float setpoint;
	float rpm;
} master_task_motor_t;

void calculo_matriz_cinematica_inversa(float *vector_velocidad_lineal, float *vector_velocidad_angular);
void calculo_matriz_cinematica_directa(float *vector_velocidad_angular, float *vector_velocidad_lineal);
void calculo_error_velocidades_lineales(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal);
void seteo_parametros_vectores(float *vector_velocidad_lineal, float *vector_velocidad_angular, motor_mqtt_params_t *motor_parameters);
void seteo_datos_motor_task(float velocidad_angular, float setpoint, master_task_motor_t *motor, QueueHandle_t queue);

#endif /* COMPONENTS_KINEMATICS_KINEMATICS_H_ */

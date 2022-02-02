/*
 * pid_controller.h
 *
 *  Created on: 30 jun. 2021
 *      Author: Administrador
 */

#ifndef MAIN_PID_CONTROLLER_H_
#define MAIN_PID_CONTROLLER_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "constants.h"

#define INCLUDE_vTaskDelay 1

// TASKS defines
#define MOTOR_TASK_COUNT	4

#define TASK_A_NAME			"T_A"
#define TASK_B_NAME			"T_B"
#define TASK_C_NAME			"T_C"
#define TASK_D_NAME			"T_D"

#define TASK_STATUS_IDLE 	0
#define TASK_STATUS_WORKING 1
#define TASK_STATUS_ERROR	255

// MASTER TASK states defines
#define ST_MT_INIT					0
#define ST_MT_SEND_SETPOINTS		1
#define ST_MT_GATHER_RPM			2
#define ST_MT_CALC_RPM_COMP			3
#define ST_MT_SEND_RPM_COMPENSATED	4

/*
 * For providing working params/resources
 * to individual motor tasks, such as
 * receive queues and assigned motor
 * */
typedef struct {
    uint8_t assigned_motor;
	xQueueHandle *rpm_count_rcv_queue;
	xQueueHandle *master_queue_rcv;
    char *task_name;
} task_params_t;

/*
 * For reporting setpoint status through
 * the feedback queue
 * */
typedef struct {
	uint8_t status;
	float average_rpm;
	char *task_name;
} master_task_feedback_t;

/*
 * For sending new setpoints to motor tasks
 * */
typedef struct {
	float setpoint;
	float rpm;
	float linefllwr_prop_const[HALL_SENSOR_COUNT];
} master_task_motor_t;

/*
 * For receiving interrupt events - ENCODERS
 * */
typedef struct {
	int16_t pulses_count;
} encoder_event_t;

/*
 * For receiving interrupt events - HALL SENSORS
 * */
typedef struct {
	int16_t hall_sensor_count[4];
} line_follower_event_t;

/*
 * For the RPM queue to calculate inverse Jacobian
 * */
typedef struct {
	float rpm;
	uint8_t busy;
} rpm_queue_t;

/*
 * For PID computing
 * */
typedef struct {
	float rpm_actual;
	float rpm_destino;
	float _integral;
	float _pre_error;
	signed int output;
} PID_params_t;

/*
 * For storing motor tasks status
 * */
typedef struct {
	uint8_t status;
	uint8_t motor_direction;
	char *task_name;
} motor_task_status_t;

// function prototypes
void main_task(void *arg);
void motor_task_creator(task_params_t *param_motor, char *taskName, uint8_t assignedMotor, xQueueHandle *masterReceiveQueue, xQueueHandle *encoderLineFllwrReceiveQueue);
void PID_Compute(PID_params_t *params_in);
float calculate_average(float *rpm_buffer, uint8_t size);
void calculo_matriz_cinematica_inversa(float *vector_velocidad_lineal, float *vector_velocidad_angular);
void calculo_matriz_cinematica_directa(rpm_queue_t *vector_velocidad_angular, float *vector_velocidad_lineal);
void calculo_error_velocidades_lineales(float *velocidad_lineal, float *velocidad_lineal_real, float *delta_velocidad_lineal);

void pwm_initialize();
void pcnt_initialize(int unit, int signal_gpio_in);
void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec);
void gpio_initialize();

void restart_pulse_counter(int pcnt);
void motorSetSpeed(uint8_t selection, signed int pwm_value);
void motorStop(uint8_t selection);
#endif /* MAIN_PID_CONTROLLER_H_ */

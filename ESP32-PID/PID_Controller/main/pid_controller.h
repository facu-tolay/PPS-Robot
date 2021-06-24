/*
 * pid_controller.h
 *
 *  Created on: 30 jun. 2021
 *      Author: Administrador
 */

#ifndef MAIN_PID_CONTROLLER_H_
#define MAIN_PID_CONTROLLER_H_

#include <stdio.h>
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

#define INCLUDE_vTaskDelay 1

// PWM defines
#define MIN_PWM_VALUE	2000
#define MAX_PWM_VALUE	8191

#define PWM_LEVEL_1		MIN_PWM_VALUE
#define PWM_LEVEL_2		4500
#define PWM_LEVEL_3		5000
#define PWM_LEVEL_4		6500
#define PWM_LEVEL_5		MAX_PWM_VALUE

// TIMER defines
#define TIMER_DIVIDER				16  //  Hardware timer clock divider
#define TIMER_SCALE					(TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_RPM_MEASURE	(0.15) // intervalo de interrupcion - expresado en [s]

// GPIO defines
#define GPIO_READY_LED		23
#define GPIO_ENABLE_MOTORS	2

// ENCODER defines
#define PCNT_INPUT_SIG_IO_A		4	// Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_B		22	// Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_C		14	// Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_D		27	// Pulse Input GPIO

// SENSORES defines
#define PNCT_INPUT_SENSOR_1		39
#define PNCT_INPUT_SENSOR_2		34	
#define PNCT_INPUT_SENSOR_3		35

// MOTOR defines
#define MOT_1_A_GPIO	5
#define MOT_1_B_GPIO	18
#define MOT_2_A_GPIO	21
#define MOT_2_B_GPIO	19
#define MOT_3_A_GPIO	32
#define MOT_3_B_GPIO	33
#define MOT_4_A_GPIO	25
#define MOT_4_B_GPIO	26

#define MOT_A_SEL		0
#define MOT_B_SEL		1
#define MOT_C_SEL		2
#define MOT_D_SEL		3

// PWM defines
#define CANT_LEDC_CHANNELS	8

ledc_channel_config_t ledc_channel[CANT_LEDC_CHANNELS] = {
	{
		.channel    = LEDC_CHANNEL_0,
		.duty       = 0,
		.gpio_num   = MOT_1_A_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_1,
		.duty       = 0,
		.gpio_num   = MOT_1_B_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_2,
		.duty       = 0,
		.gpio_num   = MOT_2_A_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_3,
		.duty       = 0,
		.gpio_num   = MOT_2_B_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_4,
		.duty       = 0,
		.gpio_num   = MOT_3_A_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
	{
		.channel    = LEDC_CHANNEL_5,
		.duty       = 0,
		.gpio_num   = MOT_3_B_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
	{
		.channel    = LEDC_CHANNEL_6,
		.duty       = 0,
		.gpio_num   = MOT_4_A_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
	{
		.channel    = LEDC_CHANNEL_7,
		.duty       = 0,
		.gpio_num   = MOT_4_B_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
};

typedef struct {
    uint8_t assigned_motor;
	xQueueHandle *rpm_count_rcv_queue;
    char *task_name;
    int16_t setpoint;
} task_params_t;

typedef struct {
	int16_t pulses_count;
	int16_t sensor_count;
} motor_task_event_t;

typedef struct {
	uint8_t motor_id;
	int prop_sensor;
	unsigned int dist_actual;
	unsigned int dist_destino;
	float _integral;
	float _pre_error;
	int output;
} PID_params_t;

// function prototypes
void main_task(void *arg);
void PID_Compute(PID_params_t *params_in);

void pwm_initialize();
void pcnt_initialize(int unit, int signal_gpio_in);
void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec);
void gpio_initialize();

void motorSetSpeed(uint8_t selection, signed int pwm_value);
void motorStop(uint8_t selection);
int16_t bound_values(int16_t input);
void counter_rutine(uint8_t pcnt);
#endif /* MAIN_PID_CONTROLLER_H_ */

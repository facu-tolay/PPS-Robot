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
#define MIN_PWM_VALUE		2200
#define MAX_PWM_VALUE		8192

// TIMER defines
#define TIMER_DIVIDER          16  //  Hardware timer clock divider
#define TIMER_SCALE            (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_RPM_MEASURE	(0.15)  // sample test interval for the second timer
#define TIMER_ISR_RPM_MEASUREMENT	2

// ENCODER defines
#define PCNT_INPUT_SIG_IO_A   4  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  15 // Control GPIO HIGH=count up, LOW=count down
#define PCNT_INPUT_SIG_IO_B   22  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO_B  15 // Control GPIO HIGH=count up, LOW=count down
#define PCNT_H_LIM_VAL      10
#define PCNT_L_LIM_VAL     -10

// MOTOR defines
#define MOT_1_A_GPIO	5
#define MOT_1_B_GPIO	18
#define MOT_2_A_GPIO	19
#define MOT_2_B_GPIO	21

// GPIO defines
#define GPIO_READY_LED		23
#define GPIO_ENABLE_MOTORS	2

// PWM defines
#define CANT_LEDC_CHANNELS	4

/*
 * Prepare individual configuration
 * for each channel of LED Controller
 * by selecting:
 * - controller's channel number
 * - output duty cycle, set initially to 0
 * - GPIO number where LED is connected to
 * - speed mode, either high or low
 * - timer servicing selected channel
 *   Note: if different channels use one timer,
 *         then frequency and bit_num of these channels
 *         will be the same
 */
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
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
	{
		.channel    = LEDC_CHANNEL_3,
		.duty       = 0,
		.gpio_num   = MOT_2_B_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
};

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
    int16_t pulses_count;
    float rpm;
} timer_event_t;

// function prototypes
void main_task(void *arg);
int PID_Compute(unsigned int dist_actual, unsigned int dist_destino);

void pwm_initialize();
void pcnt_initialize(int unit, int signal_gpio_in);
void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec);
void gpio_initialize();

void motorSetSpeed(signed int pwm_value);
void motorStop();

#endif /* MAIN_PID_CONTROLLER_H_ */

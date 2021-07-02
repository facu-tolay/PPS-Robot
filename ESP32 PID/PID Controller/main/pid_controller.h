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
#include "esp_err.h"

#define SETPOINT (float)75

#define CANT_RANURAS_ENCODER 24.0
#define MIN_PWM_VALUE 1700
#define MAX_PWM_VALUE 8192
#define MAX_RPM_MOTOR 400

#define TIMER_DIVIDER          16  //  Hardware timer clock divider
#define TIMER_SCALE            (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC    (3.4179) // sample test interval for the first timer
#define TIMER_INTERVAL_RPM_MEASURE (0.75)  // sample test interval for the second timer
#define TIMER_INTERVAL_PID_UPDATE (0.21)  // sample test interval for the second timer
#define TIMER_ISR_PID_UPDATE 		1
#define TIMER_ISR_RPM_MEASUREMENT	2

#define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  15 // Control GPIO HIGH=count up, LOW=count down
#define PCNT_H_LIM_VAL      10
#define PCNT_L_LIM_VAL     -10

#define MOT_1_A_GPIO	5
#define MOT_1_B_GPIO	18
#define MOT_2_A_GPIO	19
#define MOT_2_B_GPIO	21

#define INCLUDE_vTaskDelay 1

#define CANT_LEDC_CHANNELS       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

#define CHANNEL_CH0	LEDC_CHANNEL_0
#define CHANNEL_CH1 LEDC_CHANNEL_1
#define CHANNEL_CH2 LEDC_CHANNEL_2
#define CHANNEL_CH3 LEDC_CHANNEL_3

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE

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
void PID_Compute(float dist_actual, float dist_destino);
void pwm_initialize();
void pcnt_initialize(int unit);
void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec);
void motorSetSpeed(unsigned int pwm_value);
void motorStop();
float absolute(float val);
float constrain(float val, float min, float max);

#endif /* MAIN_PID_CONTROLLER_H_ */

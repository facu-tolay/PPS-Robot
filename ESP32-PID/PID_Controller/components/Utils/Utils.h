/*
 * Utils.h
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#ifndef COMPONENTS_UTILS_UTILS_H_
#define COMPONENTS_UTILS_UTILS_H_

#include "esp_system.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "../../main/constants.h"

void pcnt_initialize(int unit, int signal_gpio_in);
void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec, void (*isr_timer_handler)(void *));
void gpio_initialize();

void restart_pulse_counter(int pcnt);
float calculate_average(float *rpm_buffer, uint8_t size);

#endif /* COMPONENTS_UTILS_UTILS_H_ */

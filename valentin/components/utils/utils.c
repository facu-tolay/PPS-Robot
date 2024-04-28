/*
 * Utils.c
 *
 *  Created on: 12 mar. 2022
 *      Author: Administrador
 */

#include "utils.h"

// Initial configuration for modules
void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec, void (*isr_timer_handler)(void *))
{
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    };

    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, isr_timer_handler, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

void pcnt_initialize(int unit, int signal_gpio_in)
{
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = signal_gpio_in,   // Set PCNT input signal and control GPIOs
        .ctrl_gpio_num = -1,                // Control pin not utilized
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
                                            // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,         // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,         // Inhibit counter(counter value will not change in this condition)
                                            // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP,       // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,       // Keep the primary counter mode if high
    };

    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(unit, 1000);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void gpio_initialize()
{
    gpio_set_direction(GPIO_READY_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_ENABLE_MOTORS, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_ENABLE_MOTORS, 0);
}

// Utils functions
void restart_pulse_counter(int pcnt)
{
    pcnt_counter_pause(pcnt);
    pcnt_counter_clear(pcnt);
    pcnt_counter_resume(pcnt);
    return;
}

float calculate_average(float *rpm_buffer, uint8_t size)
{
    if(!size)
    {
        return 0;
    }
    else
    {
        for(int i=1; i<size; i++)
        {
            rpm_buffer[0] += rpm_buffer[i];
        }

        return rpm_buffer[0]/size;
    }
}

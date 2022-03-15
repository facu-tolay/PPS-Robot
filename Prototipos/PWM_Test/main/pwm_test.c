/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define INCLUDE_vTaskDelay 1

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

#define GPIO_CH0 (13)
#define GPIO_CH1 (2)
#define GPIO_CH2 (14)
#define GPIO_CH3 (15)
#define CHANNEL_CH0	LEDC_CHANNEL_0
#define CHANNEL_CH1 LEDC_CHANNEL_1
#define CHANNEL_CH2 LEDC_CHANNEL_2
#define CHANNEL_CH3 LEDC_CHANNEL_3

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE

void app_main(void)
{
    int ch;

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer); // Set configuration of timer0 for high speed channels

    ledc_timer.speed_mode = LEDC_HS_MODE; // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);

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
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = CHANNEL_CH0,
            .duty       = 0,
            .gpio_num   = GPIO_CH0,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = CHANNEL_CH1,
            .duty       = 0,
            .gpio_num   = GPIO_CH1,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = CHANNEL_CH2,
            .duty       = 0,
            .gpio_num   = GPIO_CH2,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = CHANNEL_CH3,
            .duty       = 0,
            .gpio_num   = GPIO_CH3,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    int delay = 9;

    while (1)
    {
    	for(int i=0; i<LEDC_TEST_DUTY; i++)
		{
			for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
			{
				ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, i);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
				vTaskDelay(delay / portTICK_PERIOD_MS);
			}
		}

    	for(int i=LEDC_TEST_DUTY; i>=0; i--)
		{
			for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
			{
				ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, i);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
				vTaskDelay(delay / portTICK_PERIOD_MS);
			}
		}

    	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
		{
    		for(int i=0; i<LEDC_TEST_DUTY; i++)
			{
				ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, i);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
				vTaskDelay(delay / portTICK_PERIOD_MS);
			}
		}

		for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
		{
			for(int i=LEDC_TEST_DUTY; i>=0; i--)
			{
				ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, i);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
				vTaskDelay(delay / portTICK_PERIOD_MS);
			}
		}
    }
}


#include "motor_control.h"

ledc_channel_config_t ledc_channels[CANT_LEDC_CHANNELS] = {
    {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = MOT_A_1_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    },
    {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = MOT_A_2_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    },
    {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 0,
        .gpio_num   = MOT_B_1_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    },
    {
        .channel    = LEDC_CHANNEL_3,
        .duty       = 0,
        .gpio_num   = MOT_B_2_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    },
    {
        .channel    = LEDC_CHANNEL_4,
        .duty       = 0,
        .gpio_num   = MOT_C_1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    },
    {
        .channel    = LEDC_CHANNEL_5,
        .duty       = 0,
        .gpio_num   = MOT_C_2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    },
    {
        .channel    = LEDC_CHANNEL_6,
        .duty       = 0,
        .gpio_num   = MOT_D_1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    },
    {
        .channel    = LEDC_CHANNEL_7,
        .duty       = 0,
        .gpio_num   = MOT_D_2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    },
};

void motorInitialize()
{
    int ch;

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,   // resolution of PWM duty
        .freq_hz = 6000,                        // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,      // timer mode
        .timer_num = LEDC_TIMER_1,              // timer index
        .clk_cfg = LEDC_AUTO_CLK,               // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer); // Set configuration of timer0 for high speed channels

    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE; // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer_config(&ledc_timer);

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < CANT_LEDC_CHANNELS; ch++)
    {
        ledc_channel_config(&ledc_channels[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
}

void motorSetSpeed(uint8_t selection, signed int speed)
{
    int speed_mot_a=0;
    int speed_mot_b=0;
    uint8_t index=0;

    switch(selection)
    {
        case MOT_A_SEL:
            break;

        case MOT_B_SEL:
            index = 2;
            break;

        case MOT_C_SEL:
            index = 4;
            break;

        case MOT_D_SEL:
            index = 6;
            break;

        default:
            index = 0;
            break;
    }

    if(speed>0)
    {
        speed_mot_a = speed;
    }
    else if(speed<0)
    {
        speed_mot_b = -speed;
    }

    ledc_set_duty(ledc_channels[index].speed_mode, ledc_channels[index].channel, speed_mot_a);
    ledc_update_duty(ledc_channels[index].speed_mode, ledc_channels[index].channel);
    index++;
    ledc_set_duty(ledc_channels[index].speed_mode, ledc_channels[index].channel, speed_mot_b);
    ledc_update_duty(ledc_channels[index].speed_mode, ledc_channels[index].channel);
}

void motorStop(uint8_t selection)
{
    motorSetSpeed(selection, 0);
}

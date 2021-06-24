#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "PID-V1.0.0/pid.h"

#define SETPOINT (double)70

#define CANT_RANURAS_ENCODER 24.0
#define MAX_RPM_MOTOR 400
#define MAX_PWM_VALUE 8192
#define MIN_PWM_VALUE 90
#define RPM_PID_SCALE_FACTOR 20.48

#define TIMER_DIVIDER          16  //  Hardware timer clock divider
#define TIMER_SCALE            (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC    (3.4179) // sample test interval for the first timer
#define TIMER_INTERVAL_RPM_MEASURE (0.1)  // sample test interval for the second timer
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
/*
 * #define GPIO_CH0 (13)
#define GPIO_CH1 (2)
#define GPIO_CH2 (14)
#define GPIO_CH3 (15)
 * */

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

int16_t count = 0; // for counting pulses
float rpm;

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
    int16_t pulses_count;
} timer_event_t;

PID_TypeDef pid_motor;

xQueueHandle timer_queue;

ledc_channel_config_t ledc_channel[CANT_LEDC_CHANNELS];

/* Calculate PWM output for PID */
void PID_Compute(PID_TypeDef *uPID)
{
	double input;
	double error;
	double dInput;
	double output;

	/* ..... Compute all the working error variables ..... */
	input   = uPID->MyInput;
	error   = uPID->MySetpoint - input;
	dInput  = (input - uPID->LastInput);

	printf("my_setpoint: %f\n", uPID->MySetpoint);
	printf("my_input: %f\n", uPID->MyInput);
	printf("error: %f\n", error);
	printf("delta: %f\n", dInput);

	uPID->OutputSum += (uPID->Ki * error);

	/* ..... Add Proportional on Measurement, if P_ON_M is specified ..... */
	if (!uPID->POnE)
	{
		uPID->OutputSum -= uPID->Kp * dInput;
	}

	if (uPID->OutputSum > uPID->OutMax)
	{
		uPID->OutputSum = uPID->OutMax;
	}
	else if (uPID->OutputSum < uPID->OutMin)
	{
		uPID->OutputSum = uPID->OutMin;
	}

	/* ..... Add Proportional on Error, if P_ON_E is specified ..... */
	if (uPID->POnE)
	{
		output = uPID->Kp * error;
	}
	else
	{
		output = 0;
	}

	/* ->.... Compute Rest of PID Output ..... */
	output += uPID->OutputSum - uPID->Kd * dInput;

	if (output > uPID->OutMax)
	{
		output = uPID->OutMax;
	}
	else if (output < uPID->OutMin)
	{
		output = uPID->OutMin;
	}

	uPID->MyOutput = (uint64_t)output;

	/* ..... Remember some variables for next time ..... */
	uPID->LastInput = (uint64_t)input;

	printf("output: %f\n\n", uPID->MyOutput);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR isr_timer(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    if (timer_intr & TIMER_INTR_T1) // timer 1 -> RPM
    {
        evt.type = TIMER_ISR_RPM_MEASUREMENT;
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);

        // get pulses
        pcnt_get_counter_value(PCNT_UNIT_0, &count);
        evt.pulses_count = count;
        pcnt_counter_pause(PCNT_UNIT_0);
    	pcnt_counter_clear(PCNT_UNIT_0);
    	pcnt_counter_resume(PCNT_UNIT_0);

    	rpm = (count/CANT_RANURAS_ENCODER) * ((1/TIMER_INTERVAL_RPM_MEASURE) * 60.0);

    	// calculate new PID value
    	pid_motor.MyInput = (uint64_t)rpm;
    }
    else if(timer_intr & TIMER_INTR_T0) // timer 0 -> PID
    {
    	evt.type = TIMER_ISR_PID_UPDATE;
		timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
		evt.pulses_count = -1;
    }
    else
    {
    	evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
    timer_spinlock_give(TIMER_GROUP_0);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, isr_timer, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_initialize(int unit)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        //.counter_h_lim = PCNT_H_LIM_VAL,
        //.counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, 500);
    pcnt_filter_enable(unit);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}

static void pwm_initialize()
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
	ledc_channel_config_t channel_conf;

	channel_conf.channel    = CHANNEL_CH0;
	channel_conf.duty       = 0;
	channel_conf.gpio_num   = MOT_1_A_GPIO;
	channel_conf.speed_mode = LEDC_HS_MODE;
	channel_conf.hpoint     = 0;
	channel_conf.timer_sel  = LEDC_HS_TIMER;

	ledc_channel[0] = channel_conf;

	channel_conf.channel    = CHANNEL_CH1;
	channel_conf.duty       = 0;
	channel_conf.gpio_num   = MOT_1_B_GPIO;
	channel_conf.speed_mode = LEDC_HS_MODE;
	channel_conf.hpoint     = 0;
	channel_conf.timer_sel  = LEDC_HS_TIMER;

	ledc_channel[1] = channel_conf;
	/*
		{
			.channel    = CHANNEL_CH2,
			.duty       = 0,
			.gpio_num   = MOT_2_A_GPIO,
			.speed_mode = LEDC_LS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_LS_TIMER
		},
		{
			.channel    = CHANNEL_CH3,
			.duty       = 0,
			.gpio_num   = MOT_2_B_GPIO,
			.speed_mode = LEDC_LS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_LS_TIMER
		},
	};*/

	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < CANT_LEDC_CHANNELS; ch++)
	{
		ledc_channel_config(&ledc_channel[ch]);
	}

	// Initialize fade service.
	ledc_fade_func_install(0);
}

/*
 * The main task of this example program
 */
static void main_task(void *arg)
{
    while (1)
    {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        if(evt.type == TIMER_ISR_RPM_MEASUREMENT)
        {
        	PID_Compute(&pid_motor);
        	if(pid_motor.MyOutput > 0)
			{
				ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, (uint32_t)pid_motor.MyOutput*RPM_PID_SCALE_FACTOR);
				ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
				ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
				ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
			}
			else
			{
				ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, (uint32_t)-pid_motor.MyOutput*RPM_PID_SCALE_FACTOR);
				ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
				ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
				ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
			}
        }
    }
}

void app_main(void)
{
    int pcnt_unit = PCNT_UNIT_0;

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_initialize(pcnt_unit);
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    timer_initialize(TIMER_1, TIMER_ISR_RPM_MEASUREMENT, TIMER_INTERVAL_RPM_MEASURE);
    timer_initialize(TIMER_0, TIMER_ISR_PID_UPDATE, TIMER_INTERVAL_PID_UPDATE);
    pwm_initialize();

    pid_motor.MyInput = 0;

    pid_motor.Kp = 15;
    pid_motor.Ki = 0.5;
    pid_motor.Kd = 3;

    pid_motor.Kp = pid_motor.Kp;
    pid_motor.Ki = pid_motor.Ki * TIMER_INTERVAL_PID_UPDATE;
    pid_motor.Kd = pid_motor.Kd / TIMER_INTERVAL_PID_UPDATE;

    pid_motor.OutMax=MAX_PWM_VALUE;
    pid_motor.OutMin=MIN_PWM_VALUE;
    pid_motor.POnE = 0;

    pid_motor.MySetpoint = SETPOINT;
    // en cada llamada setear
    /*
     * myinput
     * mysetpoint
     */

    xTaskCreate(main_task, "timer_evt_task", 2048, NULL, 5, NULL);
}


#include "pid_controller.h"

int16_t count = 0; // for counting pulses
float rpm;

xQueueHandle timer_queue;
ledc_channel_config_t ledc_channel[CANT_LEDC_CHANNELS];

float K_proportional = 40;
float K_integral = 2;
float K_derivative = 0.1;

float Error_Integral = 0;
float Error_Derivative = 0;
float Previous_Error = 0;

void motorSetSpeed(unsigned int pwm_value)
{
	ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, pwm_value);
	ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
	ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
	ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
}

void motorStop()
{
	ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
	ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
	ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
	ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
}

/* Devuelve el valor absoluto de un entero */
float absolute(float val)
{
	if(val>0)
		return val;
	else
		return (val*(-1));
}

/* Limita un valor a un minimo y un maximo */
float constrain(float val, float min, float max)
{
	if((val>=min) && (val<=max))
	{
		return val;
	}

	else if (val<min)
	{
		return min;
	}

	else if (val>max)
	{
		return max;
	}

	return 0; //error
}

/* Calculate PWM output for PID */
void PID_Compute(float dist_actual, float dist_destino)
{
	// Calculate error
	double error = setpoint - pv;

	// Proportional term
	double Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	double Iout = _Ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	double Dout = _Kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout;

	// Restrict to max/min
	if( output > _max )
		output = _max;
	else if( output < _min )
		output = _min;

	// Save error to previous error
	_pre_error = error;
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
    timer_event_t evt;

    /* Retrieve the interrupt status and the counter value from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);

    if (timer_intr & TIMER_INTR_T1) // timer 1 -> RPM
	{
		timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);

		// get pulses
		pcnt_get_counter_value(PCNT_UNIT_0, &count);
		evt.pulses_count = count;
		pcnt_counter_pause(PCNT_UNIT_0);
		pcnt_counter_clear(PCNT_UNIT_0);
		pcnt_counter_resume(PCNT_UNIT_0);

		//evt.rpm = (count/CANT_RANURAS_ENCODER) * ((1/TIMER_INTERVAL_RPM_MEASURE) * 60.0);
		// calculate new PID value
	}

    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
    xQueueSendFromISR(timer_queue, &evt, NULL); // send the event data back to the main program task
    timer_spinlock_give(TIMER_GROUP_0);
    return;
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
void timer_initialize(int timer_idx, bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
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
    timer_isr_register(TIMER_GROUP_0, timer_idx, isr_timer, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
void pcnt_initialize(int unit)
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
	pcnt_set_filter_value(unit, 100);
	pcnt_filter_enable(unit);

	/* Initialize PCNT's counter */
	pcnt_counter_pause(unit);
	pcnt_counter_clear(unit);

	/* Everything is set up, now go to counting */
	pcnt_counter_resume(unit);
}

void pwm_initialize()
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
void main_task(void *arg)
{
	timer_event_t evt;
    while (1)
    {
    	xQueueReceive(timer_queue, &evt, portMAX_DELAY);
    	rpm = (evt.pulses_count/CANT_RANURAS_ENCODER) * ((1/TIMER_INTERVAL_RPM_MEASURE) * 60.0);
    	PID_Compute(rpm, SETPOINT);
    	//i++;
    	//rpm = rpm * 0.4 + evt.rpm * 0.6;
    	//if(i>=100)
		{
    		printf("RPM= %f\n", rpm);
    		//i=0;
		}
    	//vTaskDelay(100); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.
    }
}

void app_main(void)
{
    int pcnt_unit = PCNT_UNIT_0;

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_initialize(pcnt_unit);
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    timer_initialize(TIMER_1, TIMER_ISR_RPM_MEASUREMENT, TIMER_INTERVAL_RPM_MEASURE);
    pwm_initialize();

    xTaskCreate(main_task, "timer_evt_task", 2048, NULL, 5, NULL);
}


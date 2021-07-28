#include "pid_controller.h"

#define WHEEL_DIAMETER			(float)5 // in [cm]
#define CANT_RANURAS_ENCODER	(float)24
#define ONE_TURN_DISPLACEMENT	(float)15.708 // por cada vuelta de la rueda, se avanza 2.PI.r = 2 x PI x 2.5cm = 15.708 [cm]
#define DELTA_DISTANCE_PER_SLIT	(float)(ONE_TURN_DISPLACEMENT/CANT_RANURAS_ENCODER)// cuantos [cm] avanza por cada ranura

#define _Kp (float)6
#define _Ki (float)5
#define _Kd (float)35
#define _dt 	(float)TIMER_INTERVAL_RPM_MEASURE

#define SETPOINT (float)120 // in [cm]

xQueueHandle task_motor_A_queue;
xQueueHandle task_motor_B_queue;

/* Calculate PWM output for PID */
int PID_Compute(unsigned int dist_actual, unsigned int dist_destino)
{
	static float _integral;
	static float _pre_error;

	// Calculate error
	int error = dist_destino - dist_actual;

	if(!error)
	{
		_pre_error = 0;
		_integral = 0;
		return 0;
	}

	//printf("error= %d / ", error);
	float Pout = _Kp * error; // Proportional term
	//printf("Pout= %4.2f / ", Pout);

	_integral += error * _dt; // Integral term
	float Iout = _Ki * _integral;
	//printf("Iout= %4.2f / ", Iout);

	float derivative = (error - _pre_error) / _dt; // Derivative term
	float Dout = _Kd * derivative;
	//printf("Dout= %4.2f / ", Dout);

	// Calculate total output
	float output = (Pout + Iout + Dout);
	//printf("OUT_PID= %4.2f\n", output);

	// Restrict to max/min
	if(error > 0)
	{
		//motor_direction = 1;
		if(output > MAX_PWM_VALUE)
		{
			output = MAX_PWM_VALUE;
		}
		else if(output < MIN_PWM_VALUE)
		{
			output = MIN_PWM_VALUE;
		}
	}
	else if(error < 0)
	{
		//motor_direction = 0;
		if(output < -MAX_PWM_VALUE)
		{
			output = -MAX_PWM_VALUE;
		}
		else if(output > -MIN_PWM_VALUE)
		{
			output = -MIN_PWM_VALUE;
		}
	}

	// Save error to previous error
	_pre_error = error;

	return output;
}

/*
 * The main task of this example program
 */
void task_motor_A(void *arg)
{
	motor_task_event_t evt;
	float out=0;
	unsigned int motor_direction=0;

	int16_t count_sum = 0; // for accumulating pulses
	int16_t objective_count = SETPOINT / DELTA_DISTANCE_PER_SLIT;

	vTaskDelay(200);
	gpio_set_level(GPIO_READY_LED, 1);
	vTaskDelay(100);
	gpio_set_level(GPIO_READY_LED, 0);
	vTaskDelay(100);
	gpio_set_level(GPIO_ENABLE_MOTORS, 1);

    while (1)
    {
    	xQueueReceive(task_motor_A_queue, &evt, portMAX_DELAY);

    	if(motor_direction)
    	{
    		count_sum += evt.pulses_count;
    	}
    	else
    	{
    		count_sum -= evt.pulses_count;
    	}
    	out = PID_Compute(count_sum, objective_count);

    	if(out == 0)
    	{
    		motorStop(MOT_A_SEL);
    		count_sum = objective_count;
    	}
    	else
    	{
    		motorSetSpeed(MOT_A_SEL, out);
    		motor_direction = out > 0;
    	}

    	printf("TASK_A // pulses count= %u # sum= %u # count_obj= %u # OUT= %f\n", evt.pulses_count, count_sum, objective_count, out);
    	//vTaskDelay(100); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.
    }
}

void task_motor_B(void *arg)
{
	motor_task_event_t evt;
	float out=0;
	unsigned int motor_direction=0;

	int16_t count_sum = 0; // for accumulating pulses
	int16_t objective_count = SETPOINT / DELTA_DISTANCE_PER_SLIT;

	vTaskDelay(200);
	gpio_set_level(GPIO_READY_LED, 1);
	vTaskDelay(100);
	gpio_set_level(GPIO_READY_LED, 0);
	vTaskDelay(100);
	gpio_set_level(GPIO_ENABLE_MOTORS, 1);

    while (1)
    {
    	xQueueReceive(task_motor_B_queue, &evt, portMAX_DELAY);

    	if(motor_direction)
    	{
    		count_sum += evt.pulses_count;
    	}
    	else
    	{
    		count_sum -= evt.pulses_count;
    	}
    	out = PID_Compute(count_sum, objective_count);

    	if(out == 0)
    	{
    		motorStop(MOT_B_SEL);
    		count_sum = objective_count;
    	}
    	else
    	{
    		motorSetSpeed(MOT_B_SEL, out);
    		motor_direction = out > 0;
    	}

    	printf("TASK_B // pulses count= %u # sum= %u # count_obj= %u # OUT= %f\n", evt.pulses_count, count_sum, objective_count, out);
    	//vTaskDelay(100); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.
    }
}

void task_motor_C(void *arg)
{
	motor_task_event_t evt;
	float out=0;
	unsigned int motor_direction=0;

	int16_t count_sum = 0; // for accumulating pulses
	int16_t objective_count = SETPOINT / DELTA_DISTANCE_PER_SLIT;

	vTaskDelay(200);
	gpio_set_level(GPIO_READY_LED, 1);
	vTaskDelay(100);
	gpio_set_level(GPIO_READY_LED, 0);
	vTaskDelay(100);
	gpio_set_level(GPIO_ENABLE_MOTORS, 1);

    while (1)
    {
    	xQueueReceive(task_motor_C_queue, &evt, portMAX_DELAY);

    	if(motor_direction)
    	{
    		count_sum += evt.pulses_count;
    	}
    	else
    	{
    		count_sum -= evt.pulses_count;
    	}
    	out = PID_Compute(count_sum, objective_count);

    	if(out == 0)
    	{
    		motorStop(MOT_C_SEL);
    		count_sum = objective_count;
    	}
    	else
    	{
    		motorSetSpeed(MOT_C_SEL, out);
    		motor_direction = out > 0;
    	}

    	printf("TASK_C // pulses count= %u # sum= %u # count_obj= %u # OUT= %f\n", evt.pulses_count, count_sum, objective_count, out);
    	//vTaskDelay(100); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.
    }
}

void task_motor_D(void *arg)
{
	motor_task_event_t evt;
	float out=0;
	unsigned int motor_direction=0;

	int16_t count_sum = 0; // for accumulating pulses
	int16_t objective_count = SETPOINT / DELTA_DISTANCE_PER_SLIT;

	vTaskDelay(200);
	gpio_set_level(GPIO_READY_LED, 1);
	vTaskDelay(100);
	gpio_set_level(GPIO_READY_LED, 0);
	vTaskDelay(100);
	gpio_set_level(GPIO_ENABLE_MOTORS, 1);

    while (1)
    {
    	xQueueReceive(task_motor_D_queue, &evt, portMAX_DELAY);

    	if(motor_direction)
    	{
    		count_sum += evt.pulses_count;
    	}
    	else
    	{
    		count_sum -= evt.pulses_count;
    	}
    	out = PID_Compute(count_sum, objective_count);

    	if(out == 0)
    	{
    		motorStop(MOT_D_SEL);
    		count_sum = objective_count;
    	}
    	else
    	{
    		motorSetSpeed(MOT_D_SEL, out);
    		motor_direction = out > 0;
    	}

    	printf("TASK_D // pulses count= %u # sum= %u # count_obj= %u # OUT= %f\n", evt.pulses_count, count_sum, objective_count, out);
    	//vTaskDelay(100); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.
    }
}

void app_main(void)
{
    int pcnt_unit_left = PCNT_UNIT_0;
    int pcnt_unit_right = PCNT_UNIT_1;
    int pcnt_unit_front = PCNT_UNIT_2;	
    int pcnt_unit_back = PCNT_UNIT_3;


    pcnt_initialize(pcnt_unit_left, PCNT_INPUT_SIG_IO_A);
    pcnt_initialize(pcnt_unit_right, PCNT_INPUT_SIG_IO_B);
    pcnt_initialize(pcnt_unit_front, PCNT_INPUT_SIG_IO_C);
    pcnt_initialize(pcnt_unit_back, PCNT_INPUT_SIG_IO_D);

    task_motor_A_queue = xQueueCreate(10, sizeof(motor_task_event_t));
    task_motor_B_queue = xQueueCreate(10, sizeof(motor_task_event_t));
    task_motor_C_queue = xQueueCreate(10, sizeof(motor_task_event_t));
    task_motor_D_queue = xQueueCreate(10, sizeof(motor_task_event_t));
    timer_initialize(TIMER_1, TIMER_ISR_RPM_MEASUREMENT, TIMER_INTERVAL_RPM_MEASURE);

    pwm_initialize();
    gpio_initialize();

    xTaskCreate(task_motor_A, "task_motor_A", 2048, NULL, 5, NULL);
    xTaskCreate(task_motor_B, "task_motor_B", 2048, NULL, 5, NULL);
    xTaskCreate(task_motor_C, "task_motor_C", 2048, NULL, 5, NULL);
    xTaskCreate(task_motor_D, "task_motor_D", 2048, NULL, 5, NULL);

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
    motor_task_event_t evt_A;
    motor_task_event_t evt_B;
    motor_task_event_t evt_C;
    motor_task_event_t evt_D;
    int16_t count = 0; // for counting pulses

    /* Retrieve the interrupt status and the counter value from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);

    if (timer_intr & TIMER_INTR_T1) // timer 1 -> RPM
	{
		timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);

		// get pulses
		pcnt_get_counter_value(PCNT_UNIT_0, &count);
		evt_A.pulses_count = count;
		pcnt_get_counter_value(PCNT_UNIT_1, &count);
		evt_B.pulses_count = count;
		pcnt_get_counter_value(PCNT_UNIT_2, &count);
		evt_C.pulses_count = count;
		pcnt_get_counter_value(PCNT_UNIT_3, &count);
		evt_D.pulses_count = count;

		pcnt_counter_pause(PCNT_UNIT_0);
		pcnt_counter_clear(PCNT_UNIT_0);
		pcnt_counter_resume(PCNT_UNIT_0);

		pcnt_counter_pause(PCNT_UNIT_1);
		pcnt_counter_clear(PCNT_UNIT_1);
		pcnt_counter_resume(PCNT_UNIT_1);

		pcnt_counter_pause(PCNT_UNIT_2);
		pcnt_counter_clear(PCNT_UNIT_2);
		pcnt_counter_resume(PCNT_UNIT_2);

		pcnt_counter_pause(PCNT_UNIT_3);
		pcnt_counter_clear(PCNT_UNIT_3);
		pcnt_counter_resume(PCNT_UNIT_3);

	}

    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    xQueueSendFromISR(task_motor_A_queue, &evt_A, NULL); // send the event data back to the main program task
    xQueueSendFromISR(task_motor_B_queue, &evt_B, NULL); // send the event data back to the main program task
    xQueueSendFromISR(task_motor_C_queue, &evt_C, NULL); // send the event data back to the main program task
    xQueueSendFromISR(task_motor_D_queue, &evt_D, NULL); // send the event data back to the main program task

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
void pcnt_initialize(int unit, int signal_gpio_in)
{
	/* Prepare configuration for the PCNT unit */
	pcnt_config_t pcnt_config = {
		// Set PCNT input signal and control GPIOs
		.pulse_gpio_num = signal_gpio_in,
		.ctrl_gpio_num = 0,
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
		.duty_resolution = LEDC_TIMER_13_BIT,	// resolution of PWM duty
		.freq_hz = 5000,						// frequency of PWM signal
		.speed_mode = LEDC_LOW_SPEED_MODE,		// timer mode
		.timer_num = LEDC_TIMER_1,				// timer index
		.clk_cfg = LEDC_AUTO_CLK,				// Auto select the source clock
	};

	ledc_timer_config(&ledc_timer); // Set configuration of timer0 for high speed channels

	ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE; // Prepare and set configuration of timer1 for low speed channels
	ledc_timer.timer_num = LEDC_TIMER_0;
	ledc_timer_config(&ledc_timer);

	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < CANT_LEDC_CHANNELS; ch++)
	{
		ledc_channel_config(&ledc_channel[ch]);
	}

	// Initialize fade service.
	ledc_fade_func_install(0);
}

void gpio_initialize()
{
	gpio_set_direction(GPIO_READY_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_ENABLE_MOTORS, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_ENABLE_MOTORS, 0);
}

/*
 * Funcion para el control de velocidad del motor
 */
void motorSetSpeed(uint8_t selection, signed int speed)
{
	int speed_mot_a=0;
	int speed_mot_b=0;
	uint8_t index=0;

	if(selection == 1)
	{
		index=2;
	}

	if(speed>0)
	{
		speed_mot_a = speed;
	}
	else if(speed<0)
	{
		speed_mot_b = -speed;
	}

	ledc_set_duty(ledc_channel[index].speed_mode, ledc_channel[index].channel, speed_mot_a);
	ledc_update_duty(ledc_channel[index].speed_mode, ledc_channel[index].channel);
	index++;
	ledc_set_duty(ledc_channel[index].speed_mode, ledc_channel[index].channel, speed_mot_b);
	ledc_update_duty(ledc_channel[index].speed_mode, ledc_channel[index].channel);
}

void motorStop(uint8_t selection)
{
	motorSetSpeed(selection, 0);
}


#include "pid_controller.h"

#define WHEEL_DIAMETER		(float)5.08 // 2 pulgadas - expresado en [cm]
#define CANT_RANURAS_ENCODER	(float)24
#define ONE_TURN_DISPLACEMENT	(float)15.9593 // por cada vuelta de la rueda, se avanza 2.PI.r = PI x 5.08cm = 15.9593[cm]
#define DELTA_DISTANCE_PER_SLIT	(float)(ONE_TURN_DISPLACEMENT/CANT_RANURAS_ENCODER)// cuantos [cm] avanza por cada ranura

#define _Kp (float)15
#define _Ki (float)6
#define _Kd (float)15
#define _dt (float)TIMER_INTERVAL_RPM_MEASURE

#define SETPOINT (float)120 // in [cm]

// Colas donde las task de cada motor independiente recibe
// los pulsos sensados por el encoder y el seguidor de linea
xQueueHandle encoder_linefllwr_motor_A_rcv_queue;
xQueueHandle encoder_linefllwr_motor_B_rcv_queue;
xQueueHandle encoder_linefllwr_motor_C_rcv_queue;
xQueueHandle encoder_linefllwr_motor_D_rcv_queue;

// Colas donde la tarea maestra le indica
// a los motores en que setpoint colocarse
xQueueHandle master_task_motor_A_rcv_queue;
xQueueHandle master_task_motor_B_rcv_queue;
xQueueHandle master_task_motor_C_rcv_queue;
xQueueHandle master_task_motor_D_rcv_queue;

task_params_t task_params_A;
task_params_t task_params_B;
task_params_t task_params_C;
task_params_t task_params_D;

void PID_Compute(PID_params_t *params_in)
{
	uint8_t motor_id = params_in -> motor_id; 
	int prop_sensor = params_in -> prop_sensor * 20;
	float _integral = params_in -> _integral;
	float _pre_error = params_in -> _pre_error;
	unsigned int dist_destino = params_in -> dist_destino;
	unsigned int dist_actual = params_in -> dist_actual;
	float output;
	
	if (motor_id != MOT_D_SEL)
		printf("motor_id=%d  |||  prop_sensor=%d\n", motor_id, prop_sensor);
	// Proportional sensor
	float Pline = 0;

	// Calculate error
	int error = dist_destino - dist_actual;

	if(!error)
	{
		_pre_error = 0;
		_integral = 0;
		output = 0;
	}
	else
	{
		float Pout = _Kp * error; // Proportional term

		_integral += error * _dt; // Integral term
		float Iout = _Ki * _integral;

		float derivative = (error - _pre_error) / _dt; // Derivative term
		float Dout = _Kd * derivative;

		// Calculate total output
		output = (Pout + Iout + Dout) + Pline;

		// Restrict to max/min
		if(error > 0)
		{
			if(output >= MAX_PWM_VALUE)
			{
				output = MAX_PWM_VALUE;
			}
			else if(output <= MIN_PWM_VALUE)
			{
				output = MIN_PWM_VALUE;
			}
		}
		else if(error < 0)
		{
			if(output <= -MAX_PWM_VALUE)
			{
				output = -MAX_PWM_VALUE;
			}
			else if(output >= -MIN_PWM_VALUE)
			{
				output = -MIN_PWM_VALUE;
			}
		}

		// Save error to previous error
		_pre_error = error;

		//printf("%d // prop sensor= %d # Pout= %4.2f # Iout= %4.2f # Dout= %4.2f # Pline= %4.2f # OUT= %4.2f\n", motor_id, prop_sensor, Pout, Iout, Dout, Pline, output);
	}

	params_in -> _integral = _integral;
	params_in -> _pre_error = _pre_error;
	params_in -> output = output;
}

void task_motor_generic(void *arg)
{
	task_params_t *task_params = (task_params_t *) arg;
	encoder_linefllwr_event_t evt;

	int16_t count_sum = 0; // for accumulating pulses
	int16_t objective_count = (task_params->setpoint) / DELTA_DISTANCE_PER_SLIT;
	unsigned int motor_direction = objective_count > 0;

	PID_params_t params = {
			.motor_id = 0,
			._integral = 0,
			._pre_error = 0,
			.dist_actual = 0,
			.dist_destino = 0,
			.prop_sensor = 0,
			.output = 1
	};

	vTaskDelay(200);
	gpio_set_level(GPIO_READY_LED, 1);
	vTaskDelay(100);
	gpio_set_level(GPIO_READY_LED, 0);
	vTaskDelay(100);
	gpio_set_level(GPIO_ENABLE_MOTORS, 1);

    while (1)
    {
    	xQueueReceive(*(task_params->rpm_count_rcv_queue), &evt, portMAX_DELAY);
    	//if (task_params->assigned_motor == MOT_A_SEL || task_params->assigned_motor == MOT_C_SEL)
    	//	printf("motor_id = %d -> pulses_count = %d // sensor_count = %d\n", task_params->assigned_motor, evt.pulses_count, evt.sensor_count);

    	if(motor_direction)
    	{
    		count_sum += evt.pulses_count;
    	}
    	else
    	{
    		count_sum -= evt.pulses_count;
    	}
    	params.dist_actual = count_sum;
    	params.dist_destino = objective_count;
		params.prop_sensor = evt.sensor_count;
		params.motor_id = task_params->assigned_motor;

    	PID_Compute(&params);

    	if(params.output == 0)
    	{
    		motorStop(task_params->assigned_motor);
    		count_sum = objective_count;
    	}
    	else
    	{
    		motorSetSpeed(task_params->assigned_motor, params.output);
    		motor_direction = params.output > 0;
    	}

    	//printf("%s // pulses count= %d # sum= %d # count_obj= %d # OUT= %d\n", task_params->task_name, evt.pulses_count, count_sum, objective_count, params.output);
    	//vTaskDelay(100); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.
    }
}

void master_task(void *arg)
{
	// generic task generation
	task_params_A.assigned_motor = MOT_A_SEL;
	//task_params_A.setpoint = -SETPOINT;
	task_params_A.rpm_count_rcv_queue = &encoder_linefllwr_motor_A_rcv_queue;
	task_params_A.task_name = "TASK_Agen";

	task_params_B.assigned_motor = MOT_B_SEL;
	//task_params_B.setpoint = 0;
	task_params_B.rpm_count_rcv_queue = &encoder_linefllwr_motor_B_rcv_queue;
	task_params_B.task_name = "TASK_Bgen";

	task_params_C.assigned_motor = MOT_C_SEL;
	//task_params_C.setpoint = SETPOINT;
	task_params_C.rpm_count_rcv_queue = &encoder_linefllwr_motor_C_rcv_queue;
	task_params_C.task_name = "TASK_Cgen";

	task_params_D.assigned_motor = MOT_D_SEL;
	//task_params_D.setpoint = 0;
	task_params_D.rpm_count_rcv_queue = &encoder_linefllwr_motor_D_rcv_queue;
	task_params_D.task_name = "TASK_Dgen";

	xTaskCreate(task_motor_generic, "task_motor_gen", 2048, (void *)&task_params_A, 5, NULL);
	xTaskCreate(task_motor_generic, "task_motor_gen", 2048, (void *)&task_params_B, 5, NULL);
	xTaskCreate(task_motor_generic, "task_motor_gen", 2048, (void *)&task_params_C, 5, NULL);
	xTaskCreate(task_motor_generic, "task_motor_gen", 2048, (void *)&task_params_D, 5, NULL);

	//probar = {0}
	master_task_motor_t motor_A_queue =  {
			.linefllwr_prop_const = {500, 0, 500},
			.setpoint = 0
	};
	master_task_motor_t motor_B_queue;
	master_task_motor_t motor_C_queue;
	master_task_motor_t motor_D_queue;

	xQueueSend(master_task_motor_A_rcv_queue, &motor_A_queue, 0);

	while(1)
	{
		vTaskDelay(100); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.
	}
}

void app_main(void)
{
	// assignment for pulse counters
    int pcnt_encoder_left = PCNT_UNIT_0;
    int pcnt_encoder_right = PCNT_UNIT_1;
    int pcnt_encoder_front = PCNT_UNIT_2;	
    int pcnt_encoder_back = PCNT_UNIT_3;
    int pcnt_linefllwr_left = PCNT_UNIT_4;
    int pcnt_linefllwr_middle = PCNT_UNIT_5;
    int pcnt_linefllwr_right = PCNT_UNIT_6;

    pcnt_initialize(pcnt_encoder_left, PCNT_INPUT_SIG_IO_A);
    pcnt_initialize(pcnt_encoder_right, PCNT_INPUT_SIG_IO_B);
    pcnt_initialize(pcnt_encoder_front, PCNT_INPUT_SIG_IO_C);
    pcnt_initialize(pcnt_encoder_back, PCNT_INPUT_SIG_IO_D);
    pcnt_initialize(pcnt_linefllwr_middle, PNCT_INPUT_SENSOR_2);
    pcnt_initialize(pcnt_linefllwr_right, PNCT_INPUT_SENSOR_1);
    pcnt_initialize(pcnt_linefllwr_left, PNCT_INPUT_SENSOR_3);

    // initialize queues
    encoder_linefllwr_motor_A_rcv_queue = xQueueCreate(10, sizeof(encoder_linefllwr_event_t));
    encoder_linefllwr_motor_B_rcv_queue = xQueueCreate(10, sizeof(encoder_linefllwr_event_t));
    encoder_linefllwr_motor_C_rcv_queue = xQueueCreate(10, sizeof(encoder_linefllwr_event_t));
    encoder_linefllwr_motor_D_rcv_queue = xQueueCreate(10, sizeof(encoder_linefllwr_event_t));

    master_task_motor_A_rcv_queue = xQueueCreate(10, sizeof(master_task_motor_t));
    master_task_motor_B_rcv_queue = xQueueCreate(10, sizeof(master_task_motor_t));
    master_task_motor_C_rcv_queue = xQueueCreate(10, sizeof(master_task_motor_t));
    master_task_motor_D_rcv_queue = xQueueCreate(10, sizeof(master_task_motor_t));

    timer_initialize(TIMER_1, TIMER_AUTORELOAD_EN, TIMER_INTERVAL_RPM_MEASURE);

    pwm_initialize();
    gpio_initialize();

    xTaskCreate(master_task, "master_task", 2048, NULL, 10, NULL);
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
    encoder_linefllwr_event_t evt_A = {0};
    encoder_linefllwr_event_t evt_B = {0};
    encoder_linefllwr_event_t evt_C = {0};
    encoder_linefllwr_event_t evt_D = {0};

	/* Retrieve the interrupt status and the counter value from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);

    if (timer_intr & TIMER_INTR_T1) // timer 1 -> RPM
	{
		timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);

		// get pulses from encoders
		pcnt_get_counter_value(PCNT_UNIT_0, &evt_A.pulses_count);
		pcnt_get_counter_value(PCNT_UNIT_1, &evt_B.pulses_count);
		pcnt_get_counter_value(PCNT_UNIT_2, &evt_C.pulses_count);
		pcnt_get_counter_value(PCNT_UNIT_3, &evt_D.pulses_count);

		// get line follower pulses
		pcnt_get_counter_value(PCNT_UNIT_4, &evt_A.sensor_count);
		pcnt_get_counter_value(PCNT_UNIT_5, &evt_B.sensor_count);
		pcnt_get_counter_value(PCNT_UNIT_6, &evt_C.sensor_count);

		// clear and restart all counts
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

		pcnt_counter_pause(PCNT_UNIT_4);
		pcnt_counter_clear(PCNT_UNIT_4);
		pcnt_counter_resume(PCNT_UNIT_4);

		pcnt_counter_pause(PCNT_UNIT_5);
		pcnt_counter_clear(PCNT_UNIT_5);
		pcnt_counter_resume(PCNT_UNIT_5);

		pcnt_counter_pause(PCNT_UNIT_6);
		pcnt_counter_clear(PCNT_UNIT_6);
		pcnt_counter_resume(PCNT_UNIT_6);

	}

    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    xQueueSendFromISR(encoder_linefllwr_motor_A_rcv_queue, &evt_A, NULL); // send the event data back to the main program task
    xQueueSendFromISR(encoder_linefllwr_motor_B_rcv_queue, &evt_B, NULL); // send the event data back to the main program task
    xQueueSendFromISR(encoder_linefllwr_motor_C_rcv_queue, &evt_C, NULL); // send the event data back to the main program task
    xQueueSendFromISR(encoder_linefllwr_motor_D_rcv_queue, &evt_D, NULL); // send the event data back to the main program task

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
		.ctrl_gpio_num = -1, // Control pin not utilized
		.channel = PCNT_CHANNEL_0,
		.unit = unit,
		// What to do on the positive / negative edge of pulse input?
		.pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
		.neg_mode = PCNT_COUNT_DIS,   // Inhibit counter(counter value will not change in this condition)
		// What to do when control input is low or high?
		.lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
		.hctrl_mode = PCNT_MODE_KEEP, // Keep the primary counter mode if high
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
		.freq_hz = 6000,						// frequency of PWM signal
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


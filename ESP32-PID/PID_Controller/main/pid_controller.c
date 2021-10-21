#include "pid_controller.h"

#define _Kp (float) 5
#define _Ki (float) 5
#define _Kd (float) 1
#define _dt (float)TIMER_INTERVAL_RPM_MEASURE

#define SETPOINT (float) 100 // in [cm]
#define DESIRED_RPM (float)25.0

// Cola de feedback desde las motor_task hacia master_task
xQueueHandle master_task_feedback;

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
	float _integral = params_in -> _integral;
	float _pre_error = params_in -> _pre_error;
	float dist_destino = params_in -> dist_destino;
	float dist_actual = params_in -> dist_actual;
	signed int output = params_in -> output;

	// Calculate error
	float error = dist_destino - dist_actual;

	if((error >= 0 && error <= 1.0) || (error <= 0 && error >= -1))
	{
		//_pre_error = 0;
		//_integral = 0;
		//output = 0;
	}
	else
	{
		float Pout = _Kp * error; // Proportional term

		_integral += error * _dt; // Integral term
		float Iout = _Ki * _integral;

		float derivative = (error - _pre_error) / _dt; // Derivative term
		float Dout = _Kd * derivative;

		// Calculate total output
		output = Pout + Iout + Dout;

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

		//printf("err=%4.2f/ pre=%4.2f/ Pout=%4.2f/ Iout=%4.2f/ Dout=%4.2f/ OUT=%d\n", error, _pre_error, Pout, Iout, Dout, output);

		// Save error to previous error
		_pre_error = error;
	}

	params_in -> _integral = _integral;
	params_in -> _pre_error = _pre_error;
	params_in -> output = output;
}

void task_motor(void *arg)
{
	task_params_t *task_params = (task_params_t *) arg;

	master_task_feedback_t master_feedback = {
		.status = TASK_STATUS_IDLE,
		.task_name = task_params->task_name
	};

	// calculo de RPM
	float rpm = 0;
	float rpm_ant = 0;
	float rpm_calc = 0;
	int16_t hold_up_count = 0;

	encoder_linefllwr_event_t evt_interrupt;
	master_task_motor_t evt_master_queue_rcv;

	float desired_rpm = 0;
	uint8_t motor_direction = 0;
	int16_t objective_count = 0;
	int16_t count_sum = 0;
	uint16_t measure_count = 0;

	PID_params_t params = {
			._integral = 0,
			._pre_error = 0,
			.dist_actual = 0,
			.dist_destino = 0,
			.output = 0
	};

    while (1)
    {
    	// receive from interrupt (encoder, line follower)
    	if(xQueueReceive(*(task_params->rpm_count_rcv_queue), &evt_interrupt, 10) == pdTRUE)
    	{
			if(evt_interrupt.pulses_count >= MIN_RPM_PULSE_COUNT)
			{
				rpm = (evt_interrupt.pulses_count/CANT_RANURAS_ENCODER) * (1/TIMER_INTERVAL_RPM_MEASURE) * 60.0;// rpm
				rpm_calc = (rpm + rpm_ant) / 2;
				rpm_ant = rpm;
			}
			else
			{
				hold_up_count += evt_interrupt.pulses_count;
				measure_count++;

				if(measure_count > 15 && hold_up_count < MIN_RPM_PULSE_COUNT) // detecting zero rpm
				{
					rpm_calc = 0;
					rpm_ant = rpm;
					hold_up_count = 0;
					measure_count = 0;
				}
				else if(hold_up_count >= MIN_RPM_PULSE_COUNT)
				{
					rpm = (hold_up_count/CANT_RANURAS_ENCODER) * (1/(measure_count*TIMER_INTERVAL_RPM_MEASURE)) * 60.0;// rpm
					rpm_calc = (rpm + rpm_ant) / 2;
					rpm_calc = motor_direction > 0? (rpm + rpm_ant) / 2 : -(rpm + rpm_ant) / 2;
					rpm_ant = rpm;
					hold_up_count = 0;
					measure_count = 0;
				}
			}

			params.dist_destino = desired_rpm;
			params.dist_actual = rpm_calc;
			PID_Compute(&params);

			printf("rpm %s: %4.3f - out: %d\n", task_params->task_name, rpm_calc, params.output);

			motorSetSpeed(task_params->assigned_motor, params.output);
    	}

    	// receive from master task
    	if(xQueueReceive(*(task_params->master_queue_rcv), &evt_master_queue_rcv, 10) == pdTRUE)
    	{
    		objective_count = evt_master_queue_rcv.setpoint / DELTA_DISTANCE_PER_SLIT;
    		desired_rpm = evt_master_queue_rcv.rpm;
    		motor_direction = desired_rpm > 0? 1 : 0;
			measure_count = 0;
			count_sum = 0;

			params._integral=0;
			params._pre_error=0;

    		master_feedback.status = TASK_STATUS_WORKING;
			xQueueSend(master_task_feedback, &master_feedback, 0);

    		printf("<%s> SETPOINT UPDATED!\n", task_params->task_name);
    	}
    }
}

void master_task(void *arg)
{
	master_task_feedback_t feedback_rcv = {0};

	motor_task_status_t tasks_status[TASK_COUNT] = {
			{
				.status = TASK_STATUS_IDLE,
				.task_name = TASK_A_NAME
			},
			{
				.status = TASK_STATUS_IDLE,
				.task_name = TASK_B_NAME
			},
			{
				.status = TASK_STATUS_IDLE,
				.task_name = TASK_C_NAME
			},
			{
				.status = TASK_STATUS_IDLE,
				.task_name = TASK_D_NAME
			},
	};

	// generic task generation
	motor_task_creator(&task_params_A, TASK_A_NAME, MOT_A_SEL, &master_task_motor_A_rcv_queue, &encoder_linefllwr_motor_A_rcv_queue);
	//motor_task_creator(&task_params_B, TASK_B_NAME, MOT_B_SEL, &master_task_motor_B_rcv_queue, &encoder_linefllwr_motor_B_rcv_queue);
	//motor_task_creator(&task_params_C, TASK_C_NAME, MOT_C_SEL, &master_task_motor_C_rcv_queue, &encoder_linefllwr_motor_C_rcv_queue);
	//motor_task_creator(&task_params_D, TASK_D_NAME, MOT_D_SEL, &master_task_motor_D_rcv_queue, &encoder_linefllwr_motor_D_rcv_queue);

	master_task_motor_t motor_A_data =  {
			.linefllwr_prop_const = {NEGATIVE_FEED_HIGH, NEGATIVE_FEED, POSITIVE_FEED, POSITIVE_FEED_HIGH},
			.setpoint = 0,
			.rpm = DESIRED_RPM
	};
	master_task_motor_t motor_B_data =  {
			.linefllwr_prop_const = {0, 0, 0, 0},
			.setpoint = -SETPOINT,
			.rpm = DESIRED_RPM
	};
	master_task_motor_t motor_C_data =  {
			.linefllwr_prop_const = {NEGATIVE_FEED_HIGH, NEGATIVE_FEED, POSITIVE_FEED, POSITIVE_FEED_HIGH},
			.setpoint = 0,
			.rpm = DESIRED_RPM
	};
	master_task_motor_t motor_D_data =  {
			.linefllwr_prop_const = {0, 0, 0, 0},
			.setpoint = SETPOINT,
			.rpm = DESIRED_RPM
	};

	vTaskDelay(200);
	gpio_set_level(GPIO_READY_LED, 1);
	vTaskDelay(100);
	gpio_set_level(GPIO_READY_LED, 0);
	vTaskDelay(100);
	gpio_set_level(GPIO_ENABLE_MOTORS, 1);

	// send setpoints
	//for(int i=0; i<2; i++)
	{
		//if(tasks_status[0].status == TASK_STATUS_IDLE)
		{
			xQueueSend(master_task_motor_A_rcv_queue, &motor_A_data, 0);
		}
		//else if(tasks_status[1].status == TASK_STATUS_IDLE)
		{
			xQueueSend(master_task_motor_B_rcv_queue, &motor_B_data, 0);
		}
		//else if(tasks_status[2].status == TASK_STATUS_IDLE)
		{
			xQueueSend(master_task_motor_C_rcv_queue, &motor_C_data, 0);
		}
		//else if(tasks_status[3].status == TASK_STATUS_IDLE)
		{
			xQueueSend(master_task_motor_D_rcv_queue, &motor_D_data, 0);
		}
	}

	while(1)
	{
		vTaskDelay(500 / portTICK_PERIOD_MS); // a veces es necesario meter un delay para dejar que otras tareas se ejecuten.

		if(xQueueReceive(master_task_feedback, &feedback_rcv, 10) == pdTRUE) // rcv feedback from motor tasks
		{
			for(int i=0; i<TASK_COUNT; i++)
			{
				if(strcmp(feedback_rcv.task_name, tasks_status[i].task_name)==0)
				{
					tasks_status[i].status = feedback_rcv.status;

					printf("TASK <%s> STATUS UPDATED! status [%d]\n", feedback_rcv.task_name, feedback_rcv.status);
				}
			}
		}
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
    int pcnt_linefllwr_middle_0 = PCNT_UNIT_5;
    int pcnt_linefllwr_middle_1 = PCNT_UNIT_6;
    int pcnt_linefllwr_right = PCNT_UNIT_7;

    pcnt_initialize(pcnt_encoder_left, PCNT_INPUT_SIG_IO_A);
    pcnt_initialize(pcnt_encoder_right, PCNT_INPUT_SIG_IO_B);
    pcnt_initialize(pcnt_encoder_front, PCNT_INPUT_SIG_IO_C);
    pcnt_initialize(pcnt_encoder_back, PCNT_INPUT_SIG_IO_D);
    pcnt_initialize(pcnt_linefllwr_middle_0, PNCT_INPUT_SENSOR_2);
    pcnt_initialize(pcnt_linefllwr_middle_1, PNCT_INPUT_SENSOR_3);
    pcnt_initialize(pcnt_linefllwr_right, PNCT_INPUT_SENSOR_4);
    pcnt_initialize(pcnt_linefllwr_left, PNCT_INPUT_SENSOR_5);

    // initialize queues
    master_task_feedback = xQueueCreate(10, sizeof(master_task_feedback_t));

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

    xTaskCreate(master_task, "master_task", 2048, NULL, 5, NULL);
    return;
}

void motor_task_creator(task_params_t *param_motor, char *taskName, uint8_t assignedMotor,
		xQueueHandle *masterReceiveQueue, xQueueHandle *encoderLineFllwrReceiveQueue)
{
	param_motor->assigned_motor = assignedMotor;
	param_motor->rpm_count_rcv_queue = encoderLineFllwrReceiveQueue;
	param_motor->master_queue_rcv = masterReceiveQueue;
	param_motor->task_name = taskName;
	xTaskCreate(task_motor, taskName, 2048, (void *)param_motor, 5, NULL);
	return;
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
    int16_t hall_sensor_count[HALL_SENSOR_COUNT]; // for hall sensors
    encoder_linefllwr_event_t evt_A = {0}; // for wheel encoders
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
		pcnt_get_counter_value(PCNT_UNIT_4, &hall_sensor_count[0]);
		pcnt_get_counter_value(PCNT_UNIT_5, &hall_sensor_count[1]);
		pcnt_get_counter_value(PCNT_UNIT_6, &hall_sensor_count[2]);
		pcnt_get_counter_value(PCNT_UNIT_7, &hall_sensor_count[3]);

		for(int i=0; i<HALL_SENSOR_COUNT; i++)
		{
			evt_A.hall_sensor_count[i] = hall_sensor_count[i];
			evt_B.hall_sensor_count[i] = hall_sensor_count[i];
			evt_C.hall_sensor_count[i] = hall_sensor_count[i];
			evt_D.hall_sensor_count[i] = hall_sensor_count[i];
		}

		// clear and restart all counts
		restart_pulse_counter(PCNT_UNIT_0);
		restart_pulse_counter(PCNT_UNIT_1);
		restart_pulse_counter(PCNT_UNIT_2);
		restart_pulse_counter(PCNT_UNIT_3);
		restart_pulse_counter(PCNT_UNIT_4);
		restart_pulse_counter(PCNT_UNIT_5);
		restart_pulse_counter(PCNT_UNIT_6);
		restart_pulse_counter(PCNT_UNIT_7);
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
	pcnt_set_filter_value(unit, 1000);
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

void restart_pulse_counter(int pcnt)
{
	pcnt_counter_pause(pcnt);
	pcnt_counter_clear(pcnt);
	pcnt_counter_resume(pcnt);
	return;
}

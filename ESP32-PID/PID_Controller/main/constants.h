#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// ROBOT body defines
#define ROBOT_RADIUS			(float)0.14

// ENCODER PARAMS defines
#define WHEEL_DIAMETER			(float)(0.0508) // 2 pulgadas - expresado en [m]
#define WHEEL_RADIUS			(float)(WHEEL_DIAMETER/2)
#define CANT_RANURAS_ENCODER	(float)(24.0)
#define ONE_TURN_DISPLACEMENT	(float)(0.159593) // por cada vuelta de la rueda, se avanza 2.PI.r = PI x 5.08cm = 15.9593[cm] = 0.159593[m]
#define DELTA_DISTANCE_PER_SLIT	(float)(0.0066497)// cuantos [m] avanza por cada ranura (ONE_TURN_DISPLACEMENT/CANT_RANURAS_ENCODER)

// PWM defines
#define MIN_PWM_VALUE	1300
#define MAX_PWM_VALUE	3500

#define CANT_LEDC_CHANNELS	8

#define POSITIVE_FEED		((float)0.75)
#define POSITIVE_FEED_HIGH	((float)2.75)
#define NEGATIVE_FEED		((float)-0.75)
#define NEGATIVE_FEED_HIGH	((float)-2.75)

// TIMER defines
#define TIMER_DIVIDER				16  //  Hardware timer clock divider
#define TIMER_SCALE					(TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_RPM_MEASURE	(0.1) // intervalo de interrupcion - expresado en [s]

// GPIO defines
#define GPIO_READY_LED		23
#define GPIO_ENABLE_MOTORS	2

// ENCODER defines
#define PCNT_INPUT_SIG_IO_A		4	// Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_B		22	// Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_C		14	// Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_D		27	// Pulse Input GPIO

// LINEFOLLOWER defines
#define LINEF_ANGULAR_COMP 		(float)3.5
#define LINEF_HYSTERESIS		0

// SENSORES defines
#define HALL_SENSOR_COUNT		3
#define PNCT_INPUT_SENSOR_1		39
#define PNCT_INPUT_SENSOR_2		34
#define PNCT_INPUT_SENSOR_3		35
#define PNCT_INPUT_SENSOR_4		36
#define PNCT_INPUT_SENSOR_5		15

// RPM defines
#define RPM_PULSES_BUFFER_SIZE				5
#define RPM_PULSES_MIN 						8
#define RPM_PULSES_MED 						15
#define RPM_PULSES_MAX 						30
#define RPM_BUFFER_SIZE						5
#define MIN_RPM_PULSE_COUNT					6
#define RPM_COMPENSATION_THRESHOLD	(float)	35.0
#define RPM_COMPENSATION			(float)	25.0

// MOTOR defines
#define MOT_1_A_GPIO	18
#define MOT_1_B_GPIO	5
#define MOT_2_A_GPIO	19
#define MOT_2_B_GPIO	21
#define MOT_3_A_GPIO	32
#define MOT_3_B_GPIO	33
#define MOT_4_A_GPIO	25
#define MOT_4_B_GPIO	26

#define MOT_A_SEL		0
#define MOT_B_SEL		1
#define MOT_C_SEL		2
#define MOT_D_SEL		3

#define DIRECTION_CW	1
#define DIRECTION_CCW	0

ledc_channel_config_t ledc_channel[CANT_LEDC_CHANNELS] = {
	{
		.channel    = LEDC_CHANNEL_0,
		.duty       = 0,
		.gpio_num   = MOT_1_A_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_1,
		.duty       = 0,
		.gpio_num   = MOT_1_B_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_2,
		.duty       = 0,
		.gpio_num   = MOT_2_A_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_3,
		.duty       = 0,
		.gpio_num   = MOT_2_B_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_0
	},
	{
		.channel    = LEDC_CHANNEL_4,
		.duty       = 0,
		.gpio_num   = MOT_3_A_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
	{
		.channel    = LEDC_CHANNEL_5,
		.duty       = 0,
		.gpio_num   = MOT_3_B_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
	{
		.channel    = LEDC_CHANNEL_6,
		.duty       = 0,
		.gpio_num   = MOT_4_A_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
	{
		.channel    = LEDC_CHANNEL_7,
		.duty       = 0,
		.gpio_num   = MOT_4_B_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.hpoint     = 0,
		.timer_sel  = LEDC_TIMER_1
	},
};

#endif

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// ROBOT body defines
#define ROBOT_RADIUS            (float)0.14

// ENCODER PARAMS defines
#define WHEEL_DIAMETER          (float)(0.0508) // 2 pulgadas - expresado en [m]
#define WHEEL_RADIUS            (float)(WHEEL_DIAMETER/2)
#define CANT_RANURAS_ENCODER    (float)(24.0)
#define ONE_TURN_DISPLACEMENT   (float)(0.159593) // por cada vuelta de la rueda, se avanza 2.PI.r = PI x 5.08cm = 15.9593[cm] = 0.159593[m]
#define DELTA_DISTANCE_PER_SLIT (float)(0.0066497)// cuantos [m] avanza por cada ranura (ONE_TURN_DISPLACEMENT/CANT_RANURAS_ENCODER)
#define MIN_DESTINATION_RADIUS  (float)0.03 // expresado en [m] - radio que se considera que "llego" al setpoint

// PWM defines
#define MIN_PWM_VALUE   1300
#define MAX_PWM_VALUE   3500

// TIMER defines
#define TIMER_DIVIDER                   16  //  Hardware timer clock divider
#define TIMER_SCALE                     (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_RPM_MEASURE      (float)(0.1) // intervalo de interrupcion - expresado en [s]
#define TIMER_INTERVAL_LINEF_MEASURE    (float)(0.1) // intervalo de interrupcion - expresado en [s]

// GPIO defines
#define GPIO_READY_LED      23
#define GPIO_ENABLE_MOTORS  2

// ENCODER defines
#define PCNT_INPUT_SIG_IO_A     4   // Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_B     22  // Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_C     14  // Pulse Input GPIO
#define PCNT_INPUT_SIG_IO_D     27  // Pulse Input GPIO

// LINEFOLLOWER defines
#define LINEF_ANGULAR_COMP      (float)5.25
#define LINEF_HYSTERESIS        0

// SENSORES defines
#define HALL_SENSOR_COUNT       3
#define PNCT_INPUT_SENSOR_1     39
#define PNCT_INPUT_SENSOR_2     34
#define PNCT_INPUT_SENSOR_3     35
#define PNCT_INPUT_SENSOR_4     36
#define PNCT_INPUT_SENSOR_5     15

// RPM defines
#define RPM_PULSES_BUFFER_SIZE              10
#define RPM_PULSES_MIN                      8
#define RPM_PULSES_MED                      15
#define RPM_PULSES_MAX                      30
#define RPM_BUFFER_SIZE                     5
#define MIN_RPM_PULSE_COUNT                 6
#define RPM_COMPENSATION_THRESHOLD  (float) 35.0
#define RPM_COMPENSATION            (float) 25.0

#define DIRECTION_CW    1
#define DIRECTION_CCW   0

/*
 * For receiving MQTT messages with new setpoint
 * */
typedef struct {
    float new_linear_velocity[3];
    float setpoint;
} mqtt_receive_setpoint_t;

/*
 * For receiving MQTT messages with new PID values
 * */
typedef struct {
    float pid_values[3];
} mqtt_receive_pid_t;

/*
 * For receiving MQTT messages with a command
 * */
typedef struct {
    uint8_t command;
} mqtt_receive_cmd_t;


#endif

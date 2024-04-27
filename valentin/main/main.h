/*
 *  Created on: 30 jun. 2021
 *      Author: Administrador
 */

#ifndef MAIN_PID_CONTROLLER_H_
#define MAIN_PID_CONTROLLER_H_

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/queue.h"
#include "constants.h"
#include "../components/motor_control/motor_control.h"
#include "../components/kinematics/kinematics.h"
#include "../components/pid/pid.h"
#include "../components/utils/utils.h"
#include "../components/mqtt_cmp/client_mqtt.h"
#include "../components/wifi/wifi.h"

#define INCLUDE_vTaskDelay 1

// TASKS defines
#define MOTOR_TASK_COUNT    4

#define TASK_A_NAME         "T_A"
#define TASK_B_NAME         "T_B"
#define TASK_C_NAME         "T_C"
#define TASK_D_NAME         "T_D"

#define TASK_STATUS_IDLE    0
#define TASK_STATUS_WORKING 1
#define TASK_STATUS_ERROR   255

// MASTER TASK states defines
#define ST_MT_INIT                  0
#define ST_MT_IDLE                  1
#define ST_MT_GATHER_RPM            2
#define ST_MT_CALC_RPM_COMP         3

// MQTT defines
#define MQTT_SEND_BUFFER    192
#define MQTT_RECV_BUFFER    64

/*
 * For providing working params/resources
 * to individual motor tasks, such as
 * receive queues and assigned motor
 * */
typedef struct {
    uint8_t assigned_motor;
    xQueueHandle *rpm_count_rcv_queue;
    xQueueHandle *master_queue_rcv;
    char *task_name;
} task_params_t;

/*
 * For reporting setpoint status through
 * the feedback queue
 * */
typedef struct {
    uint8_t status;
    float average_rpm;
    char *task_name;
} master_task_feedback_t;

/*
 * For receiving interrupt events - ENCODERS
 * */
typedef struct {
    int16_t pulses_count;
} encoder_event_t;

/*
 * For receiving interrupt events - HALL SENSORS
 * */
typedef struct {
    int16_t hall_sensor_count[4];
} line_follower_event_t;

/*
 * For the RPM queue to calculate inverse Jacobian
 * */
typedef struct {
    float rpm;
    float distance;
    uint8_t busy;
} rpm_queue_t;

// queue usada para que cada tarea de medicion de RPM envie a la motor_task padre
typedef struct {
    float rpm;
    float delta_distance;
} rpm_task_queue_t;

typedef struct {
    xQueueHandle *output_rpm_queue;
    xQueueHandle *input_interrupt_encoder_queue;
} rpm_task_parameters_t;


/*
 * For storing motor tasks status
 * */
typedef struct {
    uint8_t status;
    uint8_t motor_direction;
    char *task_name;
} motor_task_status_t;

// function prototypes
void main_task(void *arg);
void master_task(void *arg);
void master_task_AA(void *arg);
void master_task_ORG(void *arg);
void motor_task_creator(task_params_t *param_motor, char *taskName, uint8_t assignedMotor, xQueueHandle *masterReceiveQueue, xQueueHandle *encoderLineFllwrReceiveQueue);
void IRAM_ATTR isr_timer_handler_line_follower(void *param);
void IRAM_ATTR isr_timer_handler_wheel_encoder(void *param);

#endif /* MAIN_PID_CONTROLLER_H_ */

#include "main.h"

#define SETPOINT (float)0 // in [m]
#define VEL_LINEAL_X (float)0.0
#define VEL_LINEAL_Y (float)-0.28 //m/seg
#define VEL_ANGULAR (float)0.0  //rpm

// Cola de feedback desde las motor_task hacia master_task
xQueueHandle master_task_feedback;

// Cola de setpoint desde la mqtt pusblisher hacia master_task
xQueueHandle master_task_receive_setpoint_queue;

// Colas donde las task de cada motor independiente recibe
// los pulsos sensados por el encoder y el seguidor de linea
xQueueHandle encoder_motor_A_rcv_queue;
xQueueHandle encoder_motor_B_rcv_queue;
xQueueHandle encoder_motor_C_rcv_queue;
xQueueHandle encoder_motor_D_rcv_queue;

// Colas donde la tarea maestra le indica
// a los motores en que setpoint colocarse
xQueueHandle master_task_motor_A_rcv_queue;
xQueueHandle master_task_motor_B_rcv_queue;
xQueueHandle master_task_motor_C_rcv_queue;
xQueueHandle master_task_motor_D_rcv_queue;
xQueueHandle line_follower_master_rcv_queue;

task_params_t task_params_A;
task_params_t task_params_B;
task_params_t task_params_C;
task_params_t task_params_D;

//Identificador de cliente mqtt
esp_mqtt_client_handle_t mqtt_client;

int wifi_flag = 1;
static const char *TAG = "master_task";

void task_motor(void *arg)
{
    task_params_t *task_params = (task_params_t *) arg;

    master_task_feedback_t master_feedback = {
        .status = TASK_STATUS_IDLE,
        .average_rpm = 0,
        .distance = 0,
        .task_name = task_params->task_name
    };

    PID_params_t pid_params = {
        ._integral = 0,
        ._pre_error = 0,
        .rpm_actual = 0,
        .rpm_destino = 0,
        .output = 0
    };

    encoder_event_t pulse_count_event;
    motor_movement_vector_t motor_movement_vector;

    // for RPM calculation
    float rpm = 0;
    int16_t hold_up_count = 0;

    uint8_t rpm_index = 0;
    uint8_t pulses_buffer[RPM_PULSES_BUFFER_SIZE] = {0};
    uint8_t pulses_buffer_write_index = 0;
    signed int pulses_buffer_read_index = 0;
    float rpm_buffer[RPM_BUFFER_SIZE];

    // aux variables
    float desired_rpm = 0;
    uint8_t motor_direction = 0;
    int16_t objective_count = 0;
    int16_t count_sum = 0;
    int16_t prev_count_sum = 0;
    int16_t delta_count_sum = 0;
    uint16_t measure_count = 0;

     while (1)
     {
        vTaskDelay(1);

        // receive from interrupt (encoder, line follower)
        if(xQueueReceive(*(task_params->rpm_count_rcv_queue), &pulse_count_event, 0) == pdTRUE)
        {
            pulses_buffer[(pulses_buffer_write_index++)%RPM_PULSES_BUFFER_SIZE] = pulse_count_event.pulses_count;
            pulses_buffer_read_index = pulses_buffer_write_index;

            if(pulse_count_event.pulses_count <= RPM_PULSES_MIN)
            {
                for(int i=0; i<3; i++)
                {
                    hold_up_count += pulses_buffer[(RPM_PULSES_BUFFER_SIZE + pulses_buffer_read_index) % RPM_PULSES_BUFFER_SIZE];
                    measure_count = 3;

                    pulses_buffer_read_index = pulses_buffer_read_index - 1;
                }
            }
            else if(pulse_count_event.pulses_count > RPM_PULSES_MIN && pulse_count_event.pulses_count <= RPM_PULSES_MED)
            {
                for(int i=0; i<2; i++)
                {
                    hold_up_count += pulses_buffer[(RPM_PULSES_BUFFER_SIZE + pulses_buffer_read_index) % RPM_PULSES_BUFFER_SIZE];
                    measure_count = 2;

                    pulses_buffer_read_index = pulses_buffer_read_index - 1;
                }
            }
            else if(pulse_count_event.pulses_count > RPM_PULSES_MED && pulse_count_event.pulses_count <= RPM_PULSES_MAX)
            {
                hold_up_count += pulses_buffer[(RPM_PULSES_BUFFER_SIZE + pulses_buffer_read_index) % RPM_PULSES_BUFFER_SIZE];
                measure_count = 1;
            }
            else if(pulse_count_event.pulses_count > RPM_PULSES_MAX)
            {
                hold_up_count += pulses_buffer[(RPM_PULSES_BUFFER_SIZE + pulses_buffer_read_index) % RPM_PULSES_BUFFER_SIZE];
                measure_count = 1;
            }

            // calculate RPM
            rpm = (hold_up_count/CANT_RANURAS_ENCODER) * (1/(measure_count*TIMER_INTERVAL_RPM_MEASURE)) * 60.0;// rpm
            rpm = motor_direction == DIRECTION_CW ? rpm : -rpm;
            hold_up_count = 0;
            measure_count = 0;

            // store RPM into buffer for calculating average
            if(master_feedback.status == TASK_STATUS_WORKING)
            {
                rpm_buffer[rpm_index++] = rpm;
                if(rpm_index >= RPM_BUFFER_SIZE)
                {
                    rpm_index = 0;
                    delta_count_sum = count_sum - prev_count_sum;
                    prev_count_sum = count_sum;

                    // notify rpm average to master task
                    master_feedback.average_rpm = calculate_average(rpm_buffer, RPM_BUFFER_SIZE);
                    master_feedback.distance = delta_count_sum * DELTA_DISTANCE_PER_SLIT;
                    xQueueSend(master_task_feedback, &master_feedback, 0);
                }
            }

            // calculate distance setpoint
            count_sum += pulse_count_event.pulses_count;
            if(count_sum >= objective_count)
            {
                count_sum = objective_count;

                memset(rpm_buffer, 0, sizeof(rpm_buffer));
                rpm_index = 0;

                // notify master task that has arrived
                if(desired_rpm != 0)
                {
                    desired_rpm = 0;
                    master_feedback.status = TASK_STATUS_IDLE;
                    master_feedback.average_rpm = 0;
                    master_feedback.distance = 0;
                    xQueueSend(master_task_feedback, &master_feedback, 0);
                }

            }

            // calculate new PID value and set motor speed
            if(desired_rpm != 0)
            {
                pid_params.rpm_destino = desired_rpm;
                pid_params.rpm_actual = rpm;

                PID_Compute(&pid_params);
                motorSetSpeed(task_params->assigned_motor, pid_params.output);
            }
            else
            {
                motorStop(task_params->assigned_motor);
            }
        }

        // receive new pid_params from master task
        if(xQueueReceive(*(task_params->master_queue_rcv), &motor_movement_vector, 0) == pdTRUE)
        {
            desired_rpm = motor_movement_vector.rpm;
            motor_direction = desired_rpm > 0? DIRECTION_CW : DIRECTION_CCW;

            if(motor_movement_vector.setpoint >= 0)
            {
                //objective_count = motor_movement_vector.setpoint / DELTA_DISTANCE_PER_SLIT;
                objective_count = (0.3+motor_movement_vector.setpoint) / DELTA_DISTANCE_PER_SLIT; // FIXME fix temporal para evitar que el robot frene porque recorrio menos distancia y el python se bloquee esperando feedback

                measure_count = 0;
                count_sum = 0;

                pid_params._integral = 0;
                pid_params._pre_error = 0;

                memset(rpm_buffer, 0, sizeof(rpm_buffer));
                rpm_index = 0;

                if(motor_movement_vector.setpoint != 0)
                {
                    master_feedback.status = TASK_STATUS_WORKING;
                }
                else
                {
                    master_feedback.status = TASK_STATUS_IDLE;
                }
            }

            //ESP_LOGI(TAG, "[%s] new setpoint received", task_params->task_name);
        }
    }
}

void master_task(void *arg)
{
    float velocidades_lineales[VELOCITY_VECTOR_SIZE]        = {0};
    float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE] = {0};
    float delta_velocidad_lineal[VELOCITY_VECTOR_SIZE]      = {0};
    float average_distance                                  = 0;
    float velocidades_angulares_motores[MOTOR_TASK_COUNT]   = {0};
    float velocidad_angular_compensacion[MOTOR_TASK_COUNT]  = {0};
    float velocidad_angular_compensada[MOTOR_TASK_COUNT]    = {0};

    uint8_t state = ST_MT_INIT;
    uint8_t flag_stop_all_motors = 0;

    master_task_feedback_t feedback_received = {0};
    movement_vector_t movement_vector = {0};

    line_follower_event_t line_follower_received = {0};
    int line_follower_count[HALL_SENSOR_COUNT] = {0};
    uint8_t linef_hysteresis_count = 0;

    motor_task_status_t tasks_status[MOTOR_TASK_COUNT] = {
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
            }
    };

    uint8_t rpm_queue_size = 0;
    float rpm_average_array[MOTOR_TASK_COUNT] = {0};
    rpm_queue_t rpm_queue[MOTOR_TASK_COUNT] = {
            {
                .rpm = 0,
                .busy = 0
            },
            {
                .rpm = 0,
                .busy = 0
            },
            {
                .rpm = 0,
                .busy = 0
            },
            {
                .rpm = 0,
                .busy = 0
            }
    };

    // generic task generation
    motor_task_creator(&task_params_A, TASK_A_NAME, MOT_A_SEL, &master_task_motor_A_rcv_queue, &encoder_motor_A_rcv_queue);
    motor_task_creator(&task_params_B, TASK_B_NAME, MOT_B_SEL, &master_task_motor_B_rcv_queue, &encoder_motor_B_rcv_queue);
    motor_task_creator(&task_params_C, TASK_C_NAME, MOT_C_SEL, &master_task_motor_C_rcv_queue, &encoder_motor_C_rcv_queue);
    motor_task_creator(&task_params_D, TASK_D_NAME, MOT_D_SEL, &master_task_motor_D_rcv_queue, &encoder_motor_D_rcv_queue);

    motor_movement_vector_t motor_A_data =  {0};
    motor_movement_vector_t motor_B_data =  {0};
    motor_movement_vector_t motor_C_data =  {0};
    motor_movement_vector_t motor_D_data =  {0};

    while(1)
    {
        //vTaskDelay(1 / portTICK_PERIOD_MS);
        vTaskDelay(1);

        // receive line follower pulses
        if(xQueueReceive(line_follower_master_rcv_queue, &line_follower_received, 0) == pdTRUE)
        {
            if(linef_hysteresis_count > LINEF_HYSTERESIS)
            {
                for(int i=0; i<HALL_SENSOR_COUNT; i++)
                {
                    line_follower_count[i] = 0;
                }

                linef_hysteresis_count=0;
            }

            for(int i=0; i<HALL_SENSOR_COUNT; i++)
            {
                line_follower_count[i] += line_follower_received.hall_sensor_count[i];
            }
        }

        switch(state)
        {
            case ST_MT_INIT:
            {
                vTaskDelay(100);
                gpio_set_level(GPIO_READY_LED, 1);
                vTaskDelay(50);
                gpio_set_level(GPIO_READY_LED, 0);
                vTaskDelay(50);
                gpio_set_level(GPIO_ENABLE_MOTORS, 1);

                if (wifi_flag == ESP_OK)
                {
                    state = ST_MT_IDLE;
                }
                break;
            }

            case ST_MT_IDLE:
            {
                if (xQueueReceive(master_task_receive_setpoint_queue, &movement_vector, 0) == pdTRUE)
                {
                    velocidades_lineales[0] = movement_vector.velocidad_lineal_x; // FIXME esto se podria optimizar haciendo que ya venga cargado como arreglo desde mqtt
                    velocidades_lineales[1] = movement_vector.velocidad_lineal_y;
                    velocidades_lineales[2] = movement_vector.velocidad_angular;

                    calculo_matriz_cinematica_inversa(velocidades_lineales, velocidades_angulares_motores);

                    motor_A_data.rpm = velocidades_angulares_motores[0];
                    motor_B_data.rpm = velocidades_angulares_motores[1];
                    motor_C_data.rpm = velocidades_angulares_motores[2];
                    motor_D_data.rpm = velocidades_angulares_motores[3];
                    motor_A_data.setpoint = movement_vector.setpoint; // FIXME deberia calcular la distancia adecuada para cada rueda, no la misma para todas
                    motor_B_data.setpoint = movement_vector.setpoint;
                    motor_C_data.setpoint = movement_vector.setpoint;
                    motor_D_data.setpoint = movement_vector.setpoint;

                    ESP_LOGI(TAG, "AAAAAAA");
                    xQueueSend(master_task_motor_A_rcv_queue, &motor_A_data, 0);
                    xQueueSend(master_task_motor_B_rcv_queue, &motor_B_data, 0);
                    xQueueSend(master_task_motor_C_rcv_queue, &motor_C_data, 0);
                    xQueueSend(master_task_motor_D_rcv_queue, &motor_D_data, 0);
                    state = ST_MT_GATHER_RPM;
                    break;
                }

                break;
            }

            case ST_MT_GATHER_RPM:
            {
                // VER SI LA RECOLECCION DE RPM SE SIGUE HACIENDO SIN DEPENDER DE QUE ESTE EN ESTE ESTADO PARA MEJORAR
                // rcv feedback from motor tasks
                if(xQueueReceive(master_task_feedback, &feedback_received, 0) == pdTRUE)
                {
                    if(feedback_received.status == TASK_STATUS_IDLE)
                    {
                        if(!flag_stop_all_motors)
                        {
                            motor_A_data.rpm = 0;
                            motor_A_data.setpoint = 0;
                            motor_B_data.rpm = 0;
                            motor_B_data.setpoint = 0;
                            motor_C_data.rpm = 0;
                            motor_C_data.setpoint = 0;
                            motor_D_data.rpm = 0;
                            motor_D_data.setpoint = 0;

                            flag_stop_all_motors = 1;
                            state = ST_MT_SEND_RPM_COMPENSATED;
                            break;
                        }
                        break;
                    }
                    else if(feedback_received.status == TASK_STATUS_WORKING)
                    {
                        for(int i=0; i<MOTOR_TASK_COUNT; i++)
                        {
                            if(strcmp(feedback_received.task_name, tasks_status[i].task_name)==0)
                            {
                                tasks_status[i].status = feedback_received.status;
                                // allow only one element per motor per round
                                if(rpm_queue[i].busy == 0)
                                {
                                    rpm_queue_size++;
                                    rpm_queue[i].busy = 1;
                                    rpm_queue[i].rpm = feedback_received.average_rpm;
                                    rpm_queue[i].distance = feedback_received.distance;

                                    if(rpm_queue_size >= MOTOR_TASK_COUNT) // si llego al menos 1 mensaje de feedback desde cada task
                                    {
                                        rpm_queue_size = 0;

                                        for(int i=0; i<MOTOR_TASK_COUNT; i++)
                                        {
                                            rpm_queue[i].busy = 0;
                                            rpm_average_array[i] = rpm_queue[i].rpm;
                                        }

                                        state = ST_MT_CALC_RPM_COMP;
                                    }
                                }

                                break;
                            }
                        }
                    }
                }

                break;
            }

            case ST_MT_CALC_RPM_COMP:
            {
                for(int i=0; i<3; i++)
                {
                    velocidades_lineales_reales[i] = 0;
                }

                // Obtencion de las velocidades lineales reales a partir de las RPM
                calculo_matriz_cinematica_directa(rpm_average_array, velocidades_lineales_reales);

                float resultante = sqrt((pow(velocidades_lineales_reales[0], 2) + pow(velocidades_lineales_reales[1], 2)));
                float angulo = asin(velocidades_lineales_reales[1] / resultante) * 180/M_PI;

                for(int i=0; i<HALL_SENSOR_COUNT; i++)
                {
                    if ((i == 0 || i == 2) && line_follower_count[i] != 0)
                    {                       
                        if(velocidades_lineales[2] != 0)
                        {
                            if(i==0)
                            {
                                velocidades_lineales_reales[2] = velocidades_lineales_reales[2] + line_follower_count[i]*2.0*LINEF_ANGULAR_COMP;
                            }
                            else
                            {
                                velocidades_lineales_reales[2] = velocidades_lineales_reales[2] - line_follower_count[i]*2.0*LINEF_ANGULAR_COMP;
                            }
                        }
                        else
                        {
                            if(i==0)
                            {
                                velocidades_lineales_reales[2] = velocidades_lineales_reales[2] + line_follower_count[i]*1.75*LINEF_ANGULAR_COMP;
                            }
                            else
                            {
                                velocidades_lineales_reales[2] = velocidades_lineales_reales[2] - line_follower_count[i]*1.75*LINEF_ANGULAR_COMP;
                            }
                        }
                    }
                }

                calculo_error_velocidades_lineales(velocidades_lineales, velocidades_lineales_reales, delta_velocidad_lineal);
                calculo_matriz_cinematica_inversa(delta_velocidad_lineal, velocidad_angular_compensacion);

                linef_hysteresis_count++;

                for(int i=0; i<MOTOR_TASK_COUNT; i++)
                {
                    velocidad_angular_compensada[i] = rpm_queue[i].rpm - velocidad_angular_compensacion[i];
                }

                motor_A_data.rpm = velocidad_angular_compensada[0];
                motor_A_data.setpoint = -1;
                motor_B_data.rpm = velocidad_angular_compensada[1];
                motor_B_data.setpoint = -1;
                motor_C_data.rpm = velocidad_angular_compensada[2];
                motor_C_data.setpoint = -1;
                motor_D_data.rpm = velocidad_angular_compensada[3];
                motor_D_data.setpoint = -1;

                state = ST_MT_SEND_RPM_COMPENSATED;
                break;
            }

            case ST_MT_SEND_RPM_COMPENSATED:
            {
                ESP_LOGI(TAG, "BBBBBBB");
                xQueueSend(master_task_motor_A_rcv_queue, &motor_A_data, 0);
                xQueueSend(master_task_motor_B_rcv_queue, &motor_B_data, 0);
                xQueueSend(master_task_motor_C_rcv_queue, &motor_C_data, 0);
                xQueueSend(master_task_motor_D_rcv_queue, &motor_D_data, 0);

                average_distance = 0;
                for(int i=0; i<MOTOR_TASK_COUNT; i++)
                {
                    average_distance += rpm_queue[i].distance;
                    rpm_queue[i].distance = 0;
                }
                average_distance = average_distance / MOTOR_TASK_COUNT;

                send_mqtt_feedback(velocidades_lineales_reales, average_distance);

                if (flag_stop_all_motors)
                {
                    flag_stop_all_motors = 0;
                    state = ST_MT_IDLE;
                }
                else
                {
                    state = ST_MT_GATHER_RPM;
                }
                break;
            }

            default:
            {
                break;
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

    pcnt_initialize(pcnt_linefllwr_left, PNCT_INPUT_SENSOR_1);
    pcnt_initialize(pcnt_linefllwr_middle_0, PNCT_INPUT_SENSOR_3);
    pcnt_initialize(pcnt_linefllwr_middle_1, PNCT_INPUT_SENSOR_5);

    // initialize queues
    master_task_feedback = xQueueCreate(16, sizeof(master_task_feedback_t));
    master_task_receive_setpoint_queue = xQueueCreate(10, sizeof(movement_vector_t));

    encoder_motor_A_rcv_queue = xQueueCreate(16, sizeof(encoder_event_t));
    encoder_motor_B_rcv_queue = xQueueCreate(16, sizeof(encoder_event_t));
    encoder_motor_C_rcv_queue = xQueueCreate(16, sizeof(encoder_event_t));
    encoder_motor_D_rcv_queue = xQueueCreate(16, sizeof(encoder_event_t));

    master_task_motor_A_rcv_queue = xQueueCreate(1, sizeof(motor_movement_vector_t));
    master_task_motor_B_rcv_queue = xQueueCreate(1, sizeof(motor_movement_vector_t));
    master_task_motor_C_rcv_queue = xQueueCreate(1, sizeof(motor_movement_vector_t));
    master_task_motor_D_rcv_queue = xQueueCreate(1, sizeof(motor_movement_vector_t));

    line_follower_master_rcv_queue = xQueueCreate(1, sizeof(line_follower_event_t));

    timer_initialize(TIMER_0, TIMER_AUTORELOAD_EN, TIMER_INTERVAL_LINEF_MEASURE, isr_timer_handler_line_follower);
    timer_initialize(TIMER_1, TIMER_AUTORELOAD_EN, TIMER_INTERVAL_RPM_MEASURE, isr_timer_handler_wheel_encoder);

    motorInitialize();
    gpio_initialize();
    wifi_flag = wifi_initialize_station();
    mqtt_client = mqtt_app_start(&master_task_receive_setpoint_queue);

    xTaskCreate(master_task, "master_task", 2048, NULL, 10, NULL);

    return;
}

void motor_task_creator(task_params_t *param_motor, char *taskName, uint8_t assignedMotor, xQueueHandle *masterReceiveQueue, xQueueHandle *encoderLineFllwrReceiveQueue)
{
    param_motor->assigned_motor = assignedMotor;
    param_motor->rpm_count_rcv_queue = encoderLineFllwrReceiveQueue;
    param_motor->master_queue_rcv = masterReceiveQueue;
    param_motor->task_name = taskName;
    xTaskCreate(task_motor, taskName, 2048, (void *)param_motor, 5, NULL);
    return;
}

// Timer ISR handler
void IRAM_ATTR isr_timer_handler_line_follower(void *param)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) param;
    line_follower_event_t linef_evt = {0};

    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);

    if (timer_intr & TIMER_INTR_T0) // timer 1 -> line follower
    {
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, timer_idx);

        pcnt_get_counter_value(PCNT_UNIT_4, &linef_evt.hall_sensor_count[0]);
        pcnt_get_counter_value(PCNT_UNIT_5, &linef_evt.hall_sensor_count[1]);
        pcnt_get_counter_value(PCNT_UNIT_6, &linef_evt.hall_sensor_count[2]);
        pcnt_get_counter_value(PCNT_UNIT_7, &linef_evt.hall_sensor_count[3]);

        restart_pulse_counter(PCNT_UNIT_4);
        restart_pulse_counter(PCNT_UNIT_5);
        restart_pulse_counter(PCNT_UNIT_6);
        restart_pulse_counter(PCNT_UNIT_7);

        xQueueSendFromISR(line_follower_master_rcv_queue, &linef_evt, NULL);
    }

    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
    timer_spinlock_give(TIMER_GROUP_0);
    return;
}

void IRAM_ATTR isr_timer_handler_wheel_encoder(void *param)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) param;
    encoder_event_t evt_A = {0}; // for wheel encoders
    encoder_event_t evt_B = {0};
    encoder_event_t evt_C = {0};
    encoder_event_t evt_D = {0};

    /* Retrieve the interrupt status and the counter value from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);

    if (timer_intr & TIMER_INTR_T1) // timer 1 -> RPM
    {
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, timer_idx);

        pcnt_get_counter_value(PCNT_UNIT_0, &evt_A.pulses_count);
        pcnt_get_counter_value(PCNT_UNIT_1, &evt_B.pulses_count);
        pcnt_get_counter_value(PCNT_UNIT_2, &evt_C.pulses_count);
        pcnt_get_counter_value(PCNT_UNIT_3, &evt_D.pulses_count);

        restart_pulse_counter(PCNT_UNIT_0);
        restart_pulse_counter(PCNT_UNIT_1);
        restart_pulse_counter(PCNT_UNIT_2);
        restart_pulse_counter(PCNT_UNIT_3);

        xQueueSendFromISR(encoder_motor_A_rcv_queue, &evt_A, NULL); // send the event data back to the main program task
        xQueueSendFromISR(encoder_motor_B_rcv_queue, &evt_B, NULL); // send the event data back to the main program task
        xQueueSendFromISR(encoder_motor_C_rcv_queue, &evt_C, NULL); // send the event data back to the main program task
        xQueueSendFromISR(encoder_motor_D_rcv_queue, &evt_D, NULL); // send the event data back to the main program task
    }

    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
    timer_spinlock_give(TIMER_GROUP_0);
    return;
}

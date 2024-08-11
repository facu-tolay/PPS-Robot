
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include <cJSON.h>
#include "../kinematics/kinematics.h"

#define MQQT_DATA_LEN    72
#define MQQT_TOPIC_LEN   25

#define CONFIG_ROBOT_IDD "ROB_A"

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle *receive_queue);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void send_mqtt_feedback(float *delta_distance);
void send_mqtt_status_path_done();
void send_mqtt_register_request();
void forward_robot_feedback(xQueueHandle *receive_queue, movement_vector_t *motor_values);
int process_robot_feedback(const char *data, movement_vector_t *motor_values);
int register_robot(const char *data, char *robot_name);
void send_mqtt_log(char* buffer, char* topic);
void send_mqtt_feedback_only(float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE], int indice);
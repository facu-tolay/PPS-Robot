
#include <stdio.h>
#include <stdint.h>
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
#include "client_mqtt_cred.h"
#include <cJSON.h>
#include "../kinematics/kinematics.h"

#define MQQT_DATA_LEN    72

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle *ReceiveQueue);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void log_error_if_nonzero(const char *message, int error_code);
void send_log(esp_mqtt_client_handle_t client, char *log_buffer, char *topic);
void send_motor_parameters(xQueueHandle* receive_queue, motor_mqtt_params_t* motor_values);
int receive_motor_parameters(const char* const data, motor_mqtt_params_t* motor_values);






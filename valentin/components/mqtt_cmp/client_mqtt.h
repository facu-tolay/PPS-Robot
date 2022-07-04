
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
#include "json.h"
#include "../kinematics/kinematics.h"

// #define BROKER_HOST      "192.168.43.241"
#define BROKER_HOST      "192.168.0.40"
#define BROKER_PORT      1883
#define MQQT_DATA_LEN    72

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle *ReceiveQueue);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void log_error_if_nonzero(const char *message, int error_code);
void send_log(esp_mqtt_client_handle_t client, char *log_buffer, char *topic);
motor_mqtt_params_t receive_parse_parameters(char *data);
void send_motor_parameters(motor_mqtt_params_t* motor_values, xQueueHandle* receive_queue);





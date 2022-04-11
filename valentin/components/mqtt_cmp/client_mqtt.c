#include "client_mqtt.h"

static const char *TAG_1 = "mqtt_client";

xQueueHandle *master_task_mqtt_send_setpoint;
xQueueHandle *master_task_mqtt_send_pid_values;
xQueueHandle *master_task_mqtt_send_command;

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle *master_task_mqtt_receive)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = BROKER_HOST,
        .port = BROKER_PORT,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));

    master_task_mqtt_send_setpoint = master_task_mqtt_receive;

    return client;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_1, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_1, "MQTT_EVENT_CONNECTED");
            // msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);

            esp_mqtt_client_subscribe(event->client, "cmd", 0);
            ESP_LOGI(TAG_1, "sent subscribe successful to topic CMD");

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_1, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_1, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG_1, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
            {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG_1, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        case MQTT_EVENT_DATA:
        {
			ESP_LOGI(TAG_1, "MQTT_EVENT_DATA");
			char *command;
			char *payload;

			printf("DATA=%.*s\r\n", event->data_len, event->data);

			// 01.25,00.43,05.87,XX.XX
			// pid,pp.pp,ii.ii,dd.dd
			// setp,xx.xx,yy.yy,tt.tt,dd.dd
			// cmd,stop
			// cmd,start
			/*for(int i=0; i<4; i++)
			{
				if(i<=2)
				{
					strncpy(temp_value, event->data + i*6, 5);
					send_to_master_task.new_linear_velocity[i] = atof(temp_value);
					printf("Extracted value: %2.2f\n", send_to_master_task.new_linear_velocity[i]);
				}
				else
				{
					strncpy(temp_value, event->data + i*6, 5);
					send_to_master_task.setpoint = atof(temp_value);
					printf("Extracted value: %2.2f\n", send_to_master_task.setpoint);
				}
			}*/

			// get command/message type
			command = strtok(event->data, ",");
			payload = strtok(NULL, ",");

			if(strcmp(command, "pid") == 0)
			{
				mqtt_receive_pid_t new_pid_values;

				for(int i=0; i<3; i++)
				{
					strncpy(payload, event->data + i*6, 5);
					new_pid_values.pid_values[i] = atof(payload);
					printf("MQTT PID # Extracted value: %2.2f\n", new_pid_values.pid_values[i]);
				}

				xQueueSend(master_task_mqtt_send_pid_values, &new_pid_values, 0);
			}
			else if(strcmp(command, "setp") == 0)
			{
				mqtt_receive_setpoint_t new_setpoint;
				for(int i=0; i<4; i++)
				{
					if(i<=2)
					{
						strncpy(payload, event->data + i*6, 5);
						new_setpoint.new_linear_velocity[i] = atof(payload);
						printf("MQTT SETP # Extracted value: %2.2f\n", new_setpoint.new_linear_velocity[i]);
					}
					else
					{
						strncpy(payload, event->data + i*6, 5);
						new_setpoint.setpoint = atof(payload);
						printf("MQTT SETP # Extracted value: %2.2f\n", new_setpoint.setpoint);
					}
				}

				xQueueSend(master_task_mqtt_send_setpoint, &new_setpoint, 0);
			}
			else if(strcmp(command, "cmd") == 0)
			{
				mqtt_receive_cmd_t new_command;

				if(strcmp(payload, "start") == 0)
				{
					new_command.command = 1;
				}
				else if(strcmp(payload, "stop") == 0)
				{
					new_command.command = 0;
				}

				printf("MQTT CMD # Extracted value: %s\n", payload);
				xQueueSend(master_task_mqtt_send_command, &new_command, 0);
			}
			else
			{
				printf("MQTT ERROR # Command not found.\n");
			}

			break;
        }

        default:
            ESP_LOGI(TAG_1, "Other event id:%d", event->event_id);
            break;
    }
}

void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG_1, "Last error %s: 0x%x", message, error_code);
    }
}

void send_log(esp_mqtt_client_handle_t client, char *log_buffer, char *topic)
{
    if (esp_mqtt_client_publish(client, topic, log_buffer, 0, 0, 0) == -1)
    {
        ESP_LOGE(TAG_1, "error in enqueue msg");
    }
}

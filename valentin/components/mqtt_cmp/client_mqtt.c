#include "client_mqtt.h"

static const char *TAG = "mqtt_client";
esp_mqtt_client_handle_t client = NULL;
char topic_robot_id[MQQT_TOPIC_LEN] = {0};
char topic_receive_setpoint[MQQT_TOPIC_LEN] = {0};
char topic_robot_ok[MQQT_TOPIC_LEN] = {0};

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle* receive_queue)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = CONFIG_BROKER_HOST,
        .port = CONFIG_BROKER_PORT,
        .client_id = CONFIG_ROBOT_ID,
        .keepalive = 20,
        .disable_clean_session = true,
        .lwt_topic = "/topic/lwt",
        .lwt_msg = CONFIG_ROBOT_ID,
        .lwt_msg_len = 5
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void*)receive_queue));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    sprintf(topic_robot_id, "/topic/%s", mqtt_cfg.client_id);
    sprintf(topic_receive_setpoint, "/topic/setpoint/%s", mqtt_cfg.client_id);
    sprintf(topic_robot_ok, "/topic/live/%s", mqtt_cfg.client_id);

    return client;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    xQueueHandle *receive_queue = (xQueueHandle*)handler_args;
    esp_mqtt_event_handle_t event = event_data;
    movement_vector_t motor_values = {0};
    int msg_id = 0;

    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            if ((msg_id = esp_mqtt_client_publish(client, topic_robot_id, CONFIG_ROBOT_ID, 0, 0, 0)) == ESP_FAIL)
            {
                ESP_LOGE(TAG, "error in enqueue msg, msg_id=%d", msg_id);
            }
            if ((msg_id = esp_mqtt_client_subscribe(client, topic_receive_setpoint, 0)) == ESP_FAIL)
            {
                ESP_LOGE(TAG, "error in subscribe topic, msg_id=%d", msg_id);
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, topic=%s", event->topic);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            if (strcmp(topic_receive_setpoint, event->topic))
            {
                if (receive_motor_parameters(event->data, &motor_values) == ESP_OK)
                {
                    send_motor_parameters(receive_queue, &motor_values);
                }
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

void send_log(void)
{
    if (esp_mqtt_client_publish(client, topic_robot_ok, "OK", 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in enqueue msg");
    }
}

void send_motor_parameters(xQueueHandle* receive_queue, movement_vector_t* motor_values)
{
    if (xQueueSend(*receive_queue, (void*)motor_values, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "error in send motor values");
    }
}

int receive_motor_parameters(const char* data, movement_vector_t* motor_values)
{
    const cJSON *setpoint = NULL;
    const cJSON *velocidad_lineal_x = NULL;
    const cJSON *velocidad_lineal_y = NULL;
    const cJSON *velocidad_angular = NULL;
    int status = ESP_OK;

    cJSON *data_json = cJSON_Parse(data);
    if (data_json == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        //if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "error in obtain velocities, error=%s", error_ptr);
        }
        status = ESP_FAIL;
        goto end;
    }

    setpoint = cJSON_GetObjectItemCaseSensitive(data_json, "distance");
    if (cJSON_IsNumber(setpoint))
    {
        motor_values->setpoint = setpoint->valuedouble;
    }

    velocidad_lineal_x = cJSON_GetObjectItemCaseSensitive(data_json, "vx");
    if (cJSON_IsNumber(velocidad_lineal_x))
    {
        motor_values->velocidad_lineal_x = velocidad_lineal_x->valuedouble;
    }

    velocidad_lineal_y = cJSON_GetObjectItemCaseSensitive(data_json, "vy");
    if (cJSON_IsNumber(velocidad_lineal_y))
    {
        motor_values->velocidad_lineal_y = velocidad_lineal_y->valuedouble;
    }

    velocidad_angular = cJSON_GetObjectItemCaseSensitive(data_json, "vr");
    if (cJSON_IsNumber(velocidad_angular))
    {
        motor_values->velocidad_angular = velocidad_angular->valuedouble;
    }

    end:
        cJSON_Delete(data_json);
        return status;
}

        // case MQTT_EVENT_DATA:
        // {
		// 	ESP_LOGI(TAG, "MQTT_EVENT_DATA");
		// 	char *command;
		// 	char *payload;

		// 	printf("DATA=%.*s\r\n", event->data_len, event->data);

		// 	// 01.25,00.43,05.87,XX.XX
		// 	// pid,pp.pp,ii.ii,dd.dd
		// 	// setp,xx.xx,yy.yy,tt.tt,dd.dd
		// 	// cmd,stop
		// 	// cmd,start
		// 	/*for(int i=0; i<4; i++)
		// 	{
		// 		if(i<=2)
		// 		{
		// 			strncpy(temp_value, event->data + i*6, 5);
		// 			send_to_master_task.new_linear_velocity[i] = atof(temp_value);
		// 			printf("Extracted value: %2.2f\n", send_to_master_task.new_linear_velocity[i]);
		// 		}
		// 		else
		// 		{
		// 			strncpy(temp_value, event->data + i*6, 5);
		// 			send_to_master_task.setpoint = atof(temp_value);
		// 			printf("Extracted value: %2.2f\n", send_to_master_task.setpoint);
		// 		}
		// 	}*/

		// 	// get command/message type
		// 	command = strtok(event->data, ",");
		// 	payload = strtok(NULL, ",");

		// 	if(strcmp(command, "pid") == 0)
		// 	{
		// 		mqtt_receive_pid_t new_pid_values;

		// 		for(int i=0; i<3; i++)
		// 		{
		// 			strncpy(payload, event->data + i*6, 5);
		// 			new_pid_values.pid_values[i] = atof(payload);
		// 			printf("MQTT PID # Extracted value: %2.2f\n", new_pid_values.pid_values[i]);
		// 		}

		// 		xQueueSend(master_task_mqtt_send_pid_values, &new_pid_values, 0);
		// 	}
		// 	else if(strcmp(command, "setp") == 0)
		// 	{
		// 		mqtt_receive_setpoint_t new_setpoint;
		// 		for(int i=0; i<4; i++)
		// 		{
		// 			if(i<=2)
		// 			{
		// 				strncpy(payload, event->data + i*6, 5);
		// 				new_setpoint.new_linear_velocity[i] = atof(payload);
		// 				printf("MQTT SETP # Extracted value: %2.2f\n", new_setpoint.new_linear_velocity[i]);
		// 			}
		// 			else
		// 			{
		// 				strncpy(payload, event->data + i*6, 5);
		// 				new_setpoint.setpoint = atof(payload);
		// 				printf("MQTT SETP # Extracted value: %2.2f\n", new_setpoint.setpoint);
		// 			}
		// 		}

		// 		xQueueSend(master_task_mqtt_send_setpoint, &new_setpoint, 0);
		// 	}
		// 	else if(strcmp(command, "cmd") == 0)
		// 	{
		// 		mqtt_receive_cmd_t new_command;

		// 		if(strcmp(payload, "start") == 0)
		// 		{
		// 			new_command.command = 1;
		// 		}
		// 		else if(strcmp(payload, "stop") == 0)
		// 		{
		// 			new_command.command = 0;
		// 		}

		// 		printf("MQTT CMD # Extracted value: %s\n", payload);
		// 		xQueueSend(master_task_mqtt_send_command, &new_command, 0);
		// 	}
		// 	else
		// 	{
		// 		printf("MQTT ERROR # Command not found.\n");
		// 	}

		// 	break;
        // }
#include "client_mqtt.h"

static const char *TAG = "mqtt_client";
esp_mqtt_client_handle_t client = NULL;
char topic_robot_register[MQQT_TOPIC_LEN] = {0};
char topic_receive_setpoint[MQQT_TOPIC_LEN] = {0};
char topic_robot_feedback[MQQT_TOPIC_LEN] = {0};
char robot_name[10];

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle *receive_queue)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = CONFIG_BROKER_HOST,
        .port = CONFIG_BROKER_PORT,
        .client_id = CONFIG_ROBOT_IDD,
        .keepalive = 20,
        .disable_clean_session = true,
        .lwt_topic = "topic/lwt",
        .lwt_msg = CONFIG_ROBOT_IDD,
        .lwt_msg_len = 5
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void*)receive_queue)); // the last argument is used to pass data to the event handler
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    sprintf(topic_robot_register, "topic/register");
    sprintf(topic_receive_setpoint, "topic/setpoint/%s", mqtt_cfg.client_id);
    sprintf(topic_robot_feedback, "topic/live/%s", mqtt_cfg.client_id);
    return client;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    xQueueHandle *receive_queue = (xQueueHandle*)handler_args;
    esp_mqtt_event_handle_t event = event_data;
    movement_vector_t motor_values = {0};
    int msg_id = 0;
    int register_status = 0;

    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
        {
            if ((msg_id = esp_mqtt_client_subscribe(client, topic_receive_setpoint, 0)) == ESP_FAIL)
            {
                ESP_LOGE(TAG, "error in subscribe topic, msg_id=%d", msg_id);
                break;
            }

            if ((msg_id = esp_mqtt_client_subscribe(client, topic_robot_register, 0)) == ESP_FAIL)
            {
                ESP_LOGE(TAG, "error in subscribe topic, msg_id=%d", msg_id);
                break;
            }
            ESP_LOGI(TAG, "MQTT connected");
            break;
        }

        case MQTT_EVENT_DATA:
        {
            //ESP_LOGI(TAG, "MQTT_CLIENT <data>");
            if (strncmp(topic_receive_setpoint, event->topic, strlen(topic_receive_setpoint)) == 0)
            {
                if (process_robot_feedback(event->data, &motor_values) != ESP_OK)
                {
                    ESP_LOGW(TAG, "invalid new setpoint data");
                    break;
                }
                forward_robot_feedback(receive_queue, &motor_values);
            }
            else if (strncmp(topic_robot_register, event->topic, strlen(topic_robot_register)) == 0)
            {
                register_status = register_robot(event->data, robot_name);
                if (register_status == ESP_FAIL)
                {
                    ESP_LOGW(TAG, "invalid register data");
                    break;
                }
                else if(register_status == 15)
                {
                    send_mqtt_register_request();
                }
                ESP_LOGI(TAG, "registered with name <%s>", robot_name);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
        {
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
            {
                if (event->error_handle->esp_tls_last_esp_err != 0) {ESP_LOGE(TAG, "Last error <reported from esp-tls: 0x%x>", event->error_handle->esp_tls_last_esp_err);}
                if (event->error_handle->esp_tls_stack_err != 0) {ESP_LOGE(TAG, "Last error <reported from tls stack: 0x%x>", event->error_handle->esp_tls_stack_err);}
                if (event->error_handle->esp_transport_sock_errno != 0) {ESP_LOGE(TAG, "Last error <captured as transport's socket errno: 0x%x>", event->error_handle->esp_transport_sock_errno);}
                //ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
        case MQTT_EVENT_PUBLISHED:
        case MQTT_EVENT_SUBSCRIBED:
        case MQTT_EVENT_UNSUBSCRIBED:
        default:
        {
            //ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
        }
    }
}

void send_mqtt_feedback(float *delta_distance)
{
    char buffer[90];
    sprintf(buffer, "{\"dx\": %2.3f, \"dy\": %2.3f, \"dr\": %2.3f}", delta_distance[0], delta_distance[1], delta_distance[2]);

    if (esp_mqtt_client_publish(client, "topic/robot_feedback_delta", buffer, 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in publish msg");
    }
}

void send_mqtt_feedback_only(float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE], int indice)
{
    char buffer[90];
    sprintf(buffer, "{\"vr\": %2.3f}", velocidades_lineales_reales[2]);

    if (esp_mqtt_client_publish(client, "topic/z", buffer, 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in publish msg");
    }
}

void send_mqtt_status_path_done()
{
    char buffer[16];
    sprintf(buffer, "{\"status\":0}");

    if (esp_mqtt_client_publish(client, topic_robot_feedback, buffer, 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in publish msg");
    }
}

void send_mqtt_register_request()
{
    char buffer[16];
    sprintf(buffer, "{\"robot_id\":0}");

    if (esp_mqtt_client_publish(client, topic_robot_register, buffer, 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in publish msg");
    }
}

void forward_robot_feedback(xQueueHandle *receive_queue, movement_vector_t *motor_values)
{
    if (xQueueSend(*receive_queue, (void*)motor_values, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "error in send robot values");
    }
}

void send_mqtt_log(char* buffer, char* topic)
{
    if (esp_mqtt_client_publish(client, topic, buffer, 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in publish msg");
    }
}


int process_robot_feedback(const char *data, movement_vector_t *motor_values)
{
    const cJSON *setpoint = NULL;
    const cJSON *velocidad_lineal_x = NULL;
    const cJSON *velocidad_lineal_y = NULL;
    const cJSON *velocidad_angular = NULL;
    int status = ESP_OK;
    cJSON *data_json;

    data_json = cJSON_Parse(data);
    if (data_json == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "error in obtain velocities, error=%s", error_ptr);
        status = ESP_FAIL;
        goto end;
    }

    setpoint = cJSON_GetObjectItemCaseSensitive(data_json, "distance");
    if (!cJSON_IsNumber(setpoint))
    {
        status = ESP_FAIL;
        goto end;
    }
    motor_values->setpoint = setpoint->valuedouble;

    velocidad_lineal_x = cJSON_GetObjectItemCaseSensitive(data_json, "vx");
    if (!cJSON_IsNumber(velocidad_lineal_x))
    {
        status = ESP_FAIL;
        goto end;
    }
    motor_values->velocidad_lineal_x = velocidad_lineal_x->valuedouble;

    velocidad_lineal_y = cJSON_GetObjectItemCaseSensitive(data_json, "vy");
    if (!cJSON_IsNumber(velocidad_lineal_y))
    {
        status = ESP_FAIL;
        goto end;
    }
    motor_values->velocidad_lineal_y = velocidad_lineal_y->valuedouble;

    velocidad_angular = cJSON_GetObjectItemCaseSensitive(data_json, "vr");
    if (!cJSON_IsNumber(velocidad_angular))
    {
        status = ESP_FAIL;
        goto end;
    }
    motor_values->velocidad_angular = velocidad_angular->valuedouble;

    end:
        cJSON_Delete(data_json);
        return status;
}

int register_robot(const char *data, char *robot_name)
{
    const cJSON *json_robot_name = NULL;
    const cJSON *robot_id = NULL;
    const cJSON *status = NULL;
    int json_parse_status;
    cJSON *data_json;

    data_json = cJSON_Parse(data);
    if (data_json == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "error in register robot, error=%s", error_ptr);
        json_parse_status = ESP_FAIL;
        goto end;
    }

    status = cJSON_GetObjectItemCaseSensitive(data_json, "status");
    robot_id = cJSON_GetObjectItemCaseSensitive(data_json, "robot_id");
    json_robot_name = cJSON_GetObjectItemCaseSensitive(data_json, "robot_name");

    if(status != NULL)
    {
        if (!cJSON_IsNumber(status))
        {
            json_parse_status = ESP_FAIL;
            goto end;
        }

        if(status->valuedouble != 15)
        {
            ESP_LOGE(TAG, "expected register value 15");
            json_parse_status = ESP_FAIL;
            goto end;
        }

        json_parse_status = 15;
        goto end;
    }

    if(robot_id != NULL)
    {
        if (!cJSON_IsNumber(robot_id))
        {
            json_parse_status = ESP_FAIL;
            goto end;
        }

        if(robot_id->valuedouble != 0)
        {
            ESP_LOGE(TAG, "expected robot id 0");
            json_parse_status = ESP_FAIL;
            goto end;
        }
    }

    if(robot_name != NULL)
    {
        if (!cJSON_IsString(json_robot_name))
        {
            json_parse_status = ESP_FAIL;
            goto end;
        }
        strncpy(robot_name, json_robot_name->valuestring, 8); // FIXME max largo nombre robot 8 chars
    }

    json_parse_status = ESP_OK;

    end:
        cJSON_Delete(data_json);
        return json_parse_status;
}

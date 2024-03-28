#include "client_mqtt.h"

static const char *TAG = "mqtt_client";
esp_mqtt_client_handle_t client = NULL;
char topic_robot_register_id[MQQT_TOPIC_LEN] = {0};
char topic_receive_setpoint[MQQT_TOPIC_LEN] = {0};
char topic_robot_feedback[MQQT_TOPIC_LEN] = {0};

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle *receive_queue)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = CONFIG_BROKER_HOST,
        .port = CONFIG_BROKER_PORT,
        .client_id = CONFIG_ROBOT_IDD,
        .keepalive = 20,
        .disable_clean_session = true,
        .lwt_topic = "/topic/lwt",
        .lwt_msg = CONFIG_ROBOT_IDD,
        .lwt_msg_len = 5
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void*)receive_queue));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    sprintf(topic_robot_register_id, "/topic/%s", mqtt_cfg.client_id);
    sprintf(topic_receive_setpoint, "/topic/setpoint/%s", mqtt_cfg.client_id);
    sprintf(topic_robot_feedback, "/topic/live/%s", mqtt_cfg.client_id);

    return client;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    xQueueHandle *receive_queue = (xQueueHandle*)handler_args;
    esp_mqtt_event_handle_t event = event_data;
    movement_vector_t motor_values = {0};
    int msg_id = 0;

    ESP_LOGI(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);

    switch ((esp_mqtt_event_id_t)event_id)
    {
            case MQTT_EVENT_CONNECTED:
            {
                ESP_LOGI(TAG, "MQTT_CLIENT <connected>");
                if ((msg_id = esp_mqtt_client_publish(client, topic_robot_register_id, CONFIG_ROBOT_IDD, 0, 0, 0)) == ESP_FAIL)
                {
                    ESP_LOGE(TAG, "error in enqueue msg, msg_id=%d", msg_id);
                    break;
                }

                if ((msg_id = esp_mqtt_client_subscribe(client, topic_receive_setpoint, 0)) == ESP_FAIL)
                {
                    ESP_LOGE(TAG, "error in subscribe topic, msg_id=%d", msg_id);
                    break;
                }
                break;
            }

            case MQTT_EVENT_DISCONNECTED:
            {
                ESP_LOGI(TAG, "MQTT_CLIENT <disconnected>");
                break;
            }

            case MQTT_EVENT_PUBLISHED:
            {
                ESP_LOGI(TAG, "MQTT_CLIENT <published> msg_id=%d", event->msg_id);
                break;
            }

            case MQTT_EVENT_SUBSCRIBED:
            {
                ESP_LOGI(TAG, "MQTT_CLIENT <subscribed>");
                break;
            }

            case MQTT_EVENT_UNSUBSCRIBED:
            {
                ESP_LOGI(TAG, "MQTT_CLIENT <unsubscribed>");
                break;
            }

            case MQTT_EVENT_ERROR:
            {
                ESP_LOGI(TAG, "MQTT_CLIENT <error>");
                if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
                {
                    if (event->error_handle->esp_tls_last_esp_err != 0) {ESP_LOGE(TAG, "Last error <reported from esp-tls: 0x%x>", event->error_handle->esp_tls_last_esp_err);}
                    if (event->error_handle->esp_tls_stack_err != 0) {ESP_LOGE(TAG, "Last error <reported from tls stack: 0x%x>", event->error_handle->esp_tls_stack_err);}
                    if (event->error_handle->esp_transport_sock_errno != 0) {ESP_LOGE(TAG, "Last error <captured as transport's socket errno: 0x%x>", event->error_handle->esp_transport_sock_errno);}
                    ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
                }
                break;
            }

            case MQTT_EVENT_DATA:
            {
                ESP_LOGI(TAG, "MQTT_CLIENT <data>");
                if (strcmp(topic_receive_setpoint, event->topic))
                {
                    if (process_robot_feedback(event->data, &motor_values) == ESP_OK)
                    {
                        forward_robot_feedback(receive_queue, &motor_values);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "MQTT_CLIENT invalid new setpoint data");
                    }
                }
                break;
            }

            default:
            {
                ESP_LOGI(TAG, "Other event id:%d", event->event_id);
                break;
            }
    }
}

void send_log(void)
{
    if (esp_mqtt_client_publish(client, topic_robot_register_id, "OK", 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in enqueue msg");
    }
}

void send_mqtt_feedback(float velocidades_lineales_reales[VELOCITY_VECTOR_SIZE], float average_distance)
{
    char buffer[64];
    float avg_dist_x = 0;
    float avg_dist_y = average_distance;
    sprintf(buffer, "{\"dx\":%2.2f, \"vx\":%2.2f, \"dy\":%2.2f, \"vy\":%2.2f}", avg_dist_x, velocidades_lineales_reales[0], avg_dist_y, velocidades_lineales_reales[1]);

    if (esp_mqtt_client_publish(client, topic_robot_feedback, buffer, 0, 0, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "error in enqueue msg");
    }
}

void forward_robot_feedback(xQueueHandle *receive_queue, movement_vector_t *motor_values)
{
    if (xQueueSend(*receive_queue, (void*)motor_values, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "error in send robot values");
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
        //if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "error in obtain velocities, error=%s", error_ptr);
        }
        status = ESP_FAIL;
        goto end;
    }

    ESP_LOGI(TAG, "MQTT_CLIENT processing feedback");

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

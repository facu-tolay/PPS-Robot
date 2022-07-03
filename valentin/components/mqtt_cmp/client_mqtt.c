#include "client_mqtt.h"

static const char *TAG_1 = "mqtt_client";

esp_mqtt_client_handle_t mqtt_app_start(xQueueHandle* receive_queue)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = BROKER_HOST,
        .port = BROKER_PORT,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void*)receive_queue));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));

    return client;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_1, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    xQueueHandle *receive_queue = (xQueueHandle*)handler_args;
    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_1, "MQTT_EVENT_CONNECTED");
            if ((msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0)) != ESP_FAIL)
                ESP_LOGI(TAG_1, "sent subscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_1, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_1, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG_1, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_1, "MQTT_EVENT_DATA");
            receive_setpoint(receive_queue, event->data);
            break;
        default:
            ESP_LOGI(TAG_1, "Other event id:%d", event->event_id);
            break;
    }
}

void send_log(esp_mqtt_client_handle_t client, char *log_buffer, char *topic)
{
    if (esp_mqtt_client_publish(client, topic, log_buffer, 0, 0, 0) == -1)
    {
        ESP_LOGE(TAG_1, "error in enqueue msg");
    }
}

void receive_setpoint(xQueueHandle *receive_queue, char *data)
{
    motor_mqtt_params_t motor_values = {0};
    struct json_value_s *root = json_parse(data, MQQT_DATA_LEN);
    struct json_object_s *object = json_value_as_object(root);

    set_valorres_recibidos(object->start, &motor_values.setpoint);
    set_valorres_recibidos((object->start)->next, &motor_values.velocidad_lineal_x);
    set_valorres_recibidos(((object->start)->next)->next, &motor_values.velocidad_lineal_y);
    set_valorres_recibidos((((object->start)->next)->next)->next, &motor_values.velocidad_angular);

    if (xQueueSend(*receive_queue, &motor_values, 0) != pdTRUE)
        ESP_LOGE(TAG_1, "error in send motor values");
}

void set_valorres_recibidos(struct json_object_element_s* element, float* motor_value)
{
    struct json_value_s* element_value = element->value;
    struct json_number_s* element_number = json_value_as_number(element_value);
    *motor_value = atof(element_number->number);
}
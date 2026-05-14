#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mqtt_client.h"

#define LED_VERDE       GPIO_NUM_6
#define LED_AMARILLO    GPIO_NUM_7
#define LED_ROJO        GPIO_NUM_8

#define TOPIC_TEMP      "sed/GXX/sensor/temp"
#define TOPIC_HUM       "sed/GXX/sensor/hum"
#define TOPIC_ECO2      "sed/GXX/sensor/eco2"
#define TOPIC_TVOC      "sed/GXX/sensor/tvoc"

#define TOPIC_STATUS    "sed/GXX/display/status"

static const char *TAG = "NODO_VISUALIZACION";

// VARIABLES GLOBALES
static float last_temp = 0.0f;
static float last_hum = 0.0f;
static int last_eco2 = 400;
static int last_tvoc = 0;

static bool wifi_connected = false;
static bool mqtt_connected = false;
static bool reconnect_task_running = false;

static esp_mqtt_client_handle_t mqtt_client = NULL;

#define WIFI_RECONNECT_MS 10000
#define MQTT_RECONNECT_MS 15000

static void actualizar_semaforo(void)
{
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AMARILLO, 0);
    gpio_set_level(LED_ROJO, 0);

    bool critico =
        last_eco2 >= 900 ||
        last_tvoc >= 500 ||
        last_temp >= 32.0 ||
        last_hum >= 70.0 ||
        last_hum <= 25.0;

    bool moderado =
        last_eco2 >= 600 ||
        last_tvoc >= 300 ||
        last_temp >= 28.0 ||
        last_hum >= 60.0 ||
        last_hum <= 35.0;

    if (critico) {
        ESP_LOGW(TAG,
                 "Estado CRÍTICO | Temp %.2f | Hum %.2f | eCO2 %d | TVOC %d",
                 last_temp, last_hum, last_eco2, last_tvoc);
        gpio_set_level(LED_ROJO, 1);

    } else if (moderado) {
        ESP_LOGW(TAG,
                 "Estado MODERADO | Temp %.2f | Hum %.2f | eCO2 %d | TVOC %d",
                 last_temp, last_hum, last_eco2, last_tvoc);
        gpio_set_level(LED_AMARILLO, 1);

    } else {
        ESP_LOGI(TAG,
                 "Estado ÓPTIMO | Temp %.2f | Hum %.2f | eCO2 %d | TVOC %d",
                 last_temp, last_hum, last_eco2, last_tvoc);
        gpio_set_level(LED_VERDE, 1);
    }
}

static void init_leds(void)
{
    gpio_reset_pin(LED_VERDE);
    gpio_set_direction(LED_VERDE, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_AMARILLO);
    gpio_set_direction(LED_AMARILLO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_ROJO);
    gpio_set_direction(LED_ROJO, GPIO_MODE_OUTPUT);

    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AMARILLO, 0);
    gpio_set_level(LED_ROJO, 0);
}

static void wifi_reconnect_task(void *pvParameters)
{
    while (!wifi_connected) {
        ESP_LOGW(TAG, "Reintentando conexión WiFi...");

        esp_err_t ret = esp_wifi_connect();

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_connect falló: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_MS));
    }

    reconnect_task_running = false;
    vTaskDelete(NULL);
}

static void mqtt_reconnect_task(void *pvParameters)
{
    while (1) {
        if (wifi_connected && !mqtt_connected && mqtt_client != NULL) {
            ESP_LOGW(TAG, "Reintentando conexión MQTT...");
            esp_mqtt_client_reconnect(mqtt_client);
        }

        vTaskDelay(pdMS_TO_TICKS(MQTT_RECONNECT_MS));
    }
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi conectado al AP");
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = event_data;

        wifi_connected = false;
        mqtt_connected = false;

        ESP_LOGW(TAG, "WiFi desconectado. reason=%d", disc->reason);

        if (!reconnect_task_running) {
            reconnect_task_running = true;

            xTaskCreate(
                wifi_reconnect_task,
                "wifi_reconnect_task",
                4096,
                NULL,
                5,
                NULL
            );
        }
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = event_data;

        wifi_connected = true;

        ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));

        if (mqtt_client != NULL) {
            esp_mqtt_client_reconnect(mqtt_client);
        }
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL
    ));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Conectando a WiFi SSID: %s", CONFIG_ESP_WIFI_SSID);
    esp_wifi_connect();
}

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        mqtt_connected = true;

        ESP_LOGI(TAG, "Conectado al broker MQTT");

        esp_mqtt_client_publish(
            client,
            TOPIC_STATUS,
            "Online",
            0,
            1,
            1
        );

        esp_mqtt_client_subscribe(client, TOPIC_TEMP, 1);
        esp_mqtt_client_subscribe(client, TOPIC_HUM, 1);
        esp_mqtt_client_subscribe(client, TOPIC_ECO2, 1);
        esp_mqtt_client_subscribe(client, TOPIC_TVOC, 1);

        ESP_LOGI(TAG, "Suscrito a tópicos de sensores");
        break;

    case MQTT_EVENT_DATA: {
        char topic[64];
        char data[32];

        int topic_len = event->topic_len;
        int data_len = event->data_len;

        if (topic_len >= sizeof(topic)) {
            topic_len = sizeof(topic) - 1;
        }

        if (data_len >= sizeof(data)) {
            data_len = sizeof(data) - 1;
        }

        memcpy(topic, event->topic, topic_len);
        topic[topic_len] = '\0';

        memcpy(data, event->data, data_len);
        data[data_len] = '\0';

        ESP_LOGI(TAG, "MQTT recibido | %s = %s", topic, data);

        if (strcmp(topic, TOPIC_TEMP) == 0) {
            last_temp = atof(data);

        } else if (strcmp(topic, TOPIC_HUM) == 0) {
            last_hum = atof(data);

        } else if (strcmp(topic, TOPIC_ECO2) == 0) {
            last_eco2 = atoi(data);

        } else if (strcmp(topic, TOPIC_TVOC) == 0) {
            last_tvoc = atoi(data);
        }

        actualizar_semaforo();

        break;
    }

    case MQTT_EVENT_DISCONNECTED:
        mqtt_connected = false;

        ESP_LOGW(TAG, "MQTT desconectado");

        gpio_set_level(LED_VERDE, 0);
        gpio_set_level(LED_AMARILLO, 0);
        gpio_set_level(LED_ROJO, 0);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "Error MQTT");
        break;

    default:
        break;
    }
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,

        .session.last_will = {
            .topic = TOPIC_STATUS,
            .msg = "Offline",
            .msg_len = strlen("Offline"),
            .qos = 1,
            .retain = 1,
        },

        .session.keepalive = 10,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(
        client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );

    esp_mqtt_client_start(client);

    return client;
}

void app_main(void)
{
    init_leds();

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Broker configurado: %s", CONFIG_BROKER_URL);

    wifi_init_sta();

    vTaskDelay(pdMS_TO_TICKS(2000));

    mqtt_client = mqtt_app_start();

    xTaskCreate(
        mqtt_reconnect_task,
        "mqtt_reconnect_task",
        4096,
        NULL,
        5,
        NULL
    );
}
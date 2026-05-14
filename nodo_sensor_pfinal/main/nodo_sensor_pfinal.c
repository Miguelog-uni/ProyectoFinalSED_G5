#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "ultrasonic.h"

#include <stdbool.h>

// INCLUDES MQTT
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

#include "esp_ota_ops.h"
#include <inttypes.h>

#include <esp_mac.h>
#include "mender-client.h"
#include "mender-flash.h"

#define VERSION "1.0.7"

#define I2C_PORT        I2C_NUM_0
#define SDA_GPIO        GPIO_NUM_21
#define SCL_GPIO        GPIO_NUM_22

#define HC_TRIG_GPIO    GPIO_NUM_5
#define HC_ECHO_GPIO    GPIO_NUM_18

#define PRESENCE_CM     100
#define MAX_DISTANCE_CM 400

#define SI7021_ADDR     0x40
#define SI7021_RESET    0xFE
#define SI7021_TEMP     0xF3
#define SI7021_HUM      0xF5

#define SGP30_ADDR 0x58

#define SGP30_CMD_IAQ_INIT_1      0x20
#define SGP30_CMD_IAQ_INIT_2      0x03

#define SGP30_CMD_MEASURE_IAQ_1   0x20
#define SGP30_CMD_MEASURE_IAQ_2   0x08

#define WIFI_RECONNECT_MS       5000
#define MQTT_RECONNECT_MS       5000
#define TELEMETRY_PERIOD_MS     2000
#define WIFI_STARTUP_DELAY_MS   5000

#define TOPIC_TEMP      "sed/GXX/sensor/temp"
#define TOPIC_HUM       "sed/GXX/sensor/hum"
#define TOPIC_ECO2      "sed/GXX/sensor/eco2"
#define TOPIC_TVOC      "sed/GXX/sensor/tvoc"
#define TOPIC_STATUS    "sed/GXX/status"

static const char *TAG = "SISTEMA";

static bool wifi_connected = false;
static bool mqtt_connected = false;
static bool reconnect_task_running = false;

static esp_mqtt_client_handle_t mqtt_client = NULL;

static bool si7021_available = true;
static bool sgp30_available = true;
static bool ultrasonic_available = true;

static i2c_master_dev_handle_t sgp30_handle;

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t si7021_handle;
static ultrasonic_sensor_t ultrasonic;

// --- Variables globales
mender_client_config_t mender_config;
mender_client_callbacks_t mender_callbacks;
mender_keystore_t mender_identity[2];
char mender_mac_address[18];
// --- Callbacks
static mender_err_t mender_network_connect_cb(void) { return MENDER_OK; }
static mender_err_t mender_network_release_cb(void) { return MENDER_OK; }
static mender_err_t mender_auth_failure_cb(void) { return MENDER_OK; }

static mender_err_t mender_deployment_status_cb(mender_deployment_status_t status, char *desc) {
    ESP_LOGI("MENDER", "Estado del despliegue: %s", desc ? desc : "Desconocido");
    return MENDER_OK;
}

static mender_err_t mender_auth_success_cb(void) {
    ESP_LOGI("MENDER", "Autenticacion exitosa con el servidor");
    // Esto es VITAL: le dice al ESP32 que el firmware funciona y no debe volver a la versión anterior
    return mender_flash_confirm_image();
}

static mender_err_t mender_restart_cb(void) {
    ESP_LOGI("MENDER", "Reiniciando el sistema por peticion de OTA...");
    esp_restart();
    return MENDER_OK;
}

/* ===== SI7021 y SGP30 con driver I2C nuevo ===== */

static esp_err_t si7021_write_cmd(uint8_t cmd)
{
    return i2c_master_transmit(si7021_handle, &cmd, 1, pdMS_TO_TICKS(100));
}

static esp_err_t si7021_read_raw(uint8_t cmd, uint16_t *raw)
{
    uint8_t data[2];

    esp_err_t ret = i2c_master_transmit(si7021_handle, &cmd, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    ret = i2c_master_receive(si7021_handle, data, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        return ret;
    }

    *raw = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

static esp_err_t si7021_read_temperature(float *temperature)
{
    uint16_t raw;
    esp_err_t ret = si7021_read_raw(SI7021_TEMP, &raw);

    if (ret != ESP_OK) {
        return ret;
    }

    *temperature = ((175.72f * raw) / 65536.0f) - 46.85f;
    return ESP_OK;
}

static esp_err_t si7021_read_humidity(float *humidity)
{
    uint16_t raw;
    esp_err_t ret = si7021_read_raw(SI7021_HUM, &raw);

    if (ret != ESP_OK) {
        return ret;
    }

    *humidity = ((125.0f * raw) / 65536.0f) - 6.0f;

    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f) *humidity = 0.0f;

    return ESP_OK;
}

static esp_err_t sgp30_read_air_quality(uint16_t *eco2, uint16_t *tvoc)
{
    uint8_t cmd[2] = {
        SGP30_CMD_MEASURE_IAQ_1,
        SGP30_CMD_MEASURE_IAQ_2
    };

    uint8_t data[6];

    esp_err_t ret = i2c_master_transmit(
        sgp30_handle,
        cmd,
        sizeof(cmd),
        pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    ret = i2c_master_receive(
        sgp30_handle,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    *eco2 = ((uint16_t)data[0] << 8) | data[1];
    *tvoc = ((uint16_t)data[3] << 8) | data[4];

    return ESP_OK;
}

/* ===== Inicialización ===== */

static esp_err_t init_i2c_bus(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
}

static esp_err_t init_si7021(void)
{
    i2c_device_config_t si7021_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI7021_ADDR,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(
        i2c_bus_handle,
        &si7021_config,
        &si7021_handle
    );

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo añadir Si7021 al bus I2C");
        return ret;
    }

    ret = si7021_write_cmd(SI7021_RESET);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Si7021 no responde al comando RESET");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

static esp_err_t init_sgp30(void)
{
    i2c_device_config_t sgp30_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP30_ADDR,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(
        i2c_bus_handle,
        &sgp30_config,
        &sgp30_handle
    );

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo añadir SGP30 al bus I2C");
        return ret;
    }

    uint8_t cmd[2] = {
        SGP30_CMD_IAQ_INIT_1,
        SGP30_CMD_IAQ_INIT_2
    };

    ret = i2c_master_transmit(
        sgp30_handle,
        cmd,
        sizeof(cmd),
        pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SGP30 no responde a IAQ_INIT: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}


static void sensors_init(void)
{
    esp_err_t ret_i2c = init_i2c_bus();

    if (ret_i2c != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el bus I2C. Si7021 y SGP30 desactivados");
        si7021_available = false;
        sgp30_available = false;
    } else {
        esp_err_t ret_si7021 = init_si7021();

        if (ret_si7021 == ESP_OK) {
            si7021_available = true;
            ESP_LOGI(TAG, "Si7021 inicializado correctamente");
        } else {
            si7021_available = false;
            ESP_LOGW(TAG, "Si7021 no encontrado. Se continuará sin Si7021");
        }

        esp_err_t ret_sgp30 = init_sgp30();

        if (ret_sgp30 == ESP_OK) {
            sgp30_available = true;
            ESP_LOGI(TAG, "SGP30 inicializado correctamente");
        } else {
            sgp30_available = false;
            ESP_LOGW(TAG, "SGP30 no encontrado. Se continuará sin SGP30");
        }
    }

    ultrasonic.trigger_pin = HC_TRIG_GPIO;
    ultrasonic.echo_pin = HC_ECHO_GPIO;

    esp_err_t ret_ultra = ultrasonic_init(&ultrasonic);

    if (ret_ultra == ESP_OK) {
        ultrasonic_available = true;
        ESP_LOGI(TAG, "GPIO del HC-SR04 configurados. Presencia pendiente de verificar");
    } else {
        ultrasonic_available = false;
        ESP_LOGW(TAG, "No se pudieron configurar los GPIO del HC-SR04");
    }
}

// si se desconecta, intentar a los 10 segundos reconectar al wifi
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

// si se desconecta, intentar a los 15 segundos reconectar a mqtt
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

    ESP_LOGI(TAG, "Conectando a WiFi...");
    esp_wifi_connect();
}


static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        mqtt_connected = true;
        ESP_LOGI(TAG, "Conectado al broker MQTT");
        esp_mqtt_client_publish(mqtt_client, TOPIC_STATUS, "Online", 0, 1, 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT desconectado");
        break;

        default:
            break;
        }
}

static void mqtt_publish_if_connected(const char *topic, const char *payload)
{
    if (mqtt_connected && mqtt_client != NULL) {
        esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
    } else {
        ESP_LOGW(TAG, "MQTT no conectado. %s", topic);
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

    esp_mqtt_client_handle_t client =
        esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(
        client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );

    esp_mqtt_client_start(client);

    return client;
}

static void sensor_publish_task(void *pvParameters)
{
    char payload[32];

    while (1) {

        bool presence_detected = false;

        if (ultrasonic_available) {

            uint32_t distance_cm = 0;

            esp_err_t ret = ultrasonic_measure_cm(
                &ultrasonic,
                MAX_DISTANCE_CM,
                &distance_cm
            );

            if (ret == ESP_OK) {

                ESP_LOGD(TAG, "Distancia: %lu cm", distance_cm);

                if (distance_cm > 0 &&
                    distance_cm < PRESENCE_CM) {

                    presence_detected = true;

                    ESP_LOGI(TAG, "Presencia detectada a %lu cm", distance_cm);
                }
            }
        }

        /*
         * SOLO publicamos si hay presencia
         */
        if (presence_detected) {

            /* ===== Si7021 ===== */

            if (si7021_available) {

                float temperature = 0.0f;
                float humidity = 0.0f;

                if (si7021_read_temperature(&temperature) == ESP_OK &&
                    si7021_read_humidity(&humidity) == ESP_OK) {

                    snprintf(payload,
                             sizeof(payload),
                             "%.2f",
                             temperature);

                    mqtt_publish_if_connected(TOPIC_TEMP, payload);

                    snprintf(payload,
                             sizeof(payload),
                             "%.2f",
                             humidity);

                    mqtt_publish_if_connected(TOPIC_HUM, payload);

                    ESP_LOGI(TAG,
                             "Temp: %.2f | Hum: %.2f",
                             temperature,
                             humidity);
                }
            }

            /* ===== SGP30 ===== */

            if (sgp30_available) {

                uint16_t eco2 = 0;
                uint16_t tvoc = 0;

                if (sgp30_read_air_quality(&eco2, &tvoc) == ESP_OK) {

                    snprintf(payload,
                             sizeof(payload),
                             "%u",
                             eco2);

                    mqtt_publish_if_connected(TOPIC_ECO2, payload);

                    snprintf(payload,
                             sizeof(payload),
                             "%u",
                             tvoc);

                    mqtt_publish_if_connected(TOPIC_TVOC, payload);

                    ESP_LOGI(TAG,
                             "eCO2: %u | TVOC: %u",
                             eco2,
                             tvoc);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}
/* ===== MAIN ===== */

void app_main(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    printf("--- SISTEMA INICIADO ---\n");
    printf("Versión de Firmware: %s\n", VERSION);
    printf("Ejecutando desde partición: %s\n", running->label);
    printf("Dirección Offset: 0x%08" PRIx32 "\n", running->address);

    // Inicialización de los sensores
    sensors_init();

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());

        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    vTaskDelay(pdMS_TO_TICKS(WIFI_STARTUP_DELAY_MS));

    mqtt_client = mqtt_app_start();

    xTaskCreate(
        mqtt_reconnect_task,
        "mqtt_reconnect_task",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        sensor_publish_task,
        "task_telemetria",
        4096,
        NULL,
        5,
        NULL
    );

    // --- INICIO CONFIGURACIN MENDER ---
    // 1. Obtener la dirección MAC (Mender la exige como identificador único)
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(mender_mac_address, "%02x:%02x:%02x:%02x:%02x:%02x",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // 2. Identidad
    mender_identity[0].name = "mac";
    mender_identity[0].value = mender_mac_address;
    mender_identity[1].name = NULL;
    mender_identity[1].value = NULL;

    // 3. Configurar los parámetros
    mender_config.identity = mender_identity;
    mender_config.artifact_name = VERSION;
    mender_config.device_type = "esp32";
    mender_config.host = CONFIG_MENDER_SERVER_HOST;
    mender_config.tenant_token = CONFIG_MENDER_SERVER_TENANT_TOKEN;
    mender_config.authentication_poll_interval = 60;
    mender_config.update_poll_interval = 60;
    mender_config.recommissioning = false;

    // 4. Definir los callbacks obligatorios
    mender_callbacks.network_connect = mender_network_connect_cb;
    mender_callbacks.network_release = mender_network_release_cb;
    mender_callbacks.authentication_success = mender_auth_success_cb;
    mender_callbacks.authentication_failure = mender_auth_failure_cb;
    mender_callbacks.deployment_status = mender_deployment_status_cb;
    mender_callbacks.restart = mender_restart_cb;

    // 5. Arrancar el cliente Mender en segundo plano
    ESP_LOGI("MENDER", "Iniciando cliente con MAC: %s", mender_mac_address);
    if (mender_client_init(&mender_config, &mender_callbacks) == MENDER_OK) {
        mender_client_activate(); // Esto pone a Mender a funcionar en segundo plano!
    } else {
        ESP_LOGE("MENDER", "Fallo al inicializar Mender");
    }
    // --- FIN CONFIGURACIN MENDER ---
}
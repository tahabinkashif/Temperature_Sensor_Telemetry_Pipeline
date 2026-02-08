#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "mqtt_client.h"

#include "esp_log.h"


// ============================================================
// WIFI + MQTT SETTINGS (EDIT THESE)
// ============================================================
#define WIFI_SSID       "WIFI-SSID"
#define WIFI_PASS       "WIFI-PASSWORD"

#define MQTT_BROKER_URI "mqtt://IP:PORT"

#define TOPIC_CMD       "esp32/cmd"
#define TOPIC_TELEM     "esp32/telemetry"

// ============================================================
//  PACKET FORMAT (MUST MATCH STM32)
// ============================================================
#define PKT_MAGIC       0xAA55
#define PKT_VERSION     1
#define MSG_TELEMETRY   0x01

#define MAGIC_B0 0x55
#define MAGIC_B1 0xAA

#define HEADER_LEN 12
#define CRC_LEN    2

#define MAX_PAYLOAD 64
#define MAX_FRAME   (HEADER_LEN + MAX_PAYLOAD + CRC_LEN)

// UART pins / settings
#define RX_UART          UART_NUM_2
#define RX_UART_BAUD     115200
#define RX_UART_TX_PIN   GPIO_NUM_17   // not used, but required by driver
#define RX_UART_RX_PIN   GPIO_NUM_16

static const char *TAG = "STM32_MQTT";

// ============================================================
// TELEMETRY STATE (latest values)
// ============================================================
typedef struct {
    uint16_t seq;
    uint32_t ts;
    uint16_t raw_adc;
    int16_t  temp_x100;
    uint8_t  flags;
    bool     valid;
} telemetry_t;

static telemetry_t latest = {0};

// ============================================================
// STREAM CONTROL
// ============================================================
static volatile bool stream_enabled = false;

// ============================================================
// MQTT GLOBALS
// ============================================================
static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;

// ============================================================
// WIFI EVENT GROUP
// ============================================================
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ---------------- CRC16-CCITT ----------------
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
        {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else             crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline uint16_t rd_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

// ============================================================
// STREAM BUFFER (holds UART bytes)
// ============================================================
typedef struct {
    uint8_t  data[4096];
    uint32_t len;
} stream_buf_t;

static void sb_init(stream_buf_t *sb) { sb->len = 0; }

static void sb_append(stream_buf_t *sb, const uint8_t *in, uint32_t n)
{
    if (n == 0) return;

    if (sb->len + n > sizeof(sb->data))
    {
        uint32_t drop = (sb->len + n) - sizeof(sb->data);
        if (drop >= sb->len)
        {
            sb->len = 0;
        }
        else
        {
            memmove(sb->data, sb->data + drop, sb->len - drop);
            sb->len -= drop;
        }
    }

    memcpy(sb->data + sb->len, in, n);
    sb->len += n;
}

static int sb_find_magic(const stream_buf_t *sb)
{
    for (uint32_t i = 0; i + 1 < sb->len; i++)
    {
        if (sb->data[i] == MAGIC_B0 && sb->data[i + 1] == MAGIC_B1)
            return (int)i;
    }
    return -1;
}

// ============================================================
// Try parse 1 frame from buffer
// Returns true if something was consumed, false if need more bytes
// ============================================================
static bool try_parse_one(stream_buf_t *sb)
{
    int idx = sb_find_magic(sb);
    if (idx < 0)
    {
        if (sb->len > 1)
        {
            sb->data[0] = sb->data[sb->len - 1];
            sb->len = 1;
        }
        return false;
    }

    if (idx > 0)
    {
        memmove(sb->data, sb->data + idx, sb->len - idx);
        sb->len -= idx;
    }

    if (sb->len < HEADER_LEN) return false;

    uint16_t magic = rd_le16(&sb->data[0]);
    uint8_t ver = sb->data[2];
    uint8_t msg = sb->data[3];
    uint16_t plen = rd_le16(&sb->data[4]);

    if (magic != PKT_MAGIC)
    {
        memmove(sb->data, sb->data + 1, sb->len - 1);
        sb->len -= 1;
        return true;
    }

    if (plen > MAX_PAYLOAD)
    {
        ESP_LOGW(TAG, "Bad plen=%u, resync", (unsigned)plen);
        memmove(sb->data, sb->data + 2, sb->len - 2);
        sb->len -= 2;
        return true;
    }

    uint32_t total_len = HEADER_LEN + plen + CRC_LEN;
    if (sb->len < total_len) return false;

    uint16_t recv_crc = rd_le16(&sb->data[total_len - 2]);
    uint16_t calc_crc = crc16_ccitt(sb->data, total_len - 2);

    if (recv_crc != calc_crc)
    {
        ESP_LOGW(TAG, "CRC FAIL got=0x%04X calc=0x%04X",
                 (unsigned)recv_crc, (unsigned)calc_crc);

        memmove(sb->data, sb->data + 1, sb->len - 1);
        sb->len -= 1;
        return true;
    }

    uint16_t seq = rd_le16(&sb->data[6]);
    uint32_t ts  = rd_le32(&sb->data[8]);

    if (ver != PKT_VERSION)
    {
        ESP_LOGW(TAG, "Version mismatch: %u != %u", (unsigned)ver, (unsigned)PKT_VERSION);
    }

    if (msg == MSG_TELEMETRY && plen == 5)
    {
        const uint8_t *payload = &sb->data[HEADER_LEN];
        uint16_t raw_adc = rd_le16(&payload[0]);
        int16_t temp_x100 = (int16_t)rd_le16(&payload[2]);
        uint8_t flags = payload[4];

        latest.seq = seq;
        latest.ts = ts;
        latest.raw_adc = raw_adc;
        latest.temp_x100 = temp_x100;
        latest.flags = flags;
        latest.valid = true;
    }

    uint32_t remain = sb->len - total_len;
    if (remain > 0) memmove(sb->data, sb->data + total_len, remain);
    sb->len = remain;

    return true;
}

// ============================================================
// UART event-driven RX task
// ============================================================
static QueueHandle_t uart_queue;

static void uart_rx_task(void *arg)
{
    stream_buf_t sb;
    sb_init(&sb);

    uart_event_t event;
    uint8_t rx[512];

    while (1)
    {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY))
        {
            if (event.type == UART_DATA)
            {
                int n = uart_read_bytes(RX_UART, rx, event.size, 0);
                if (n > 0)
                {
                    sb_append(&sb, rx, (uint32_t)n);
                    while (try_parse_one(&sb)) { }
                }
            }
            else if (event.type == UART_FIFO_OVF)
            {
                ESP_LOGW(TAG, "UART FIFO overflow");
                uart_flush_input(RX_UART);
                xQueueReset(uart_queue);
            }
            else if (event.type == UART_BUFFER_FULL)
            {
                ESP_LOGW(TAG, "UART ring buffer full");
                uart_flush_input(RX_UART);
                xQueueReset(uart_queue);
            }
        }
    }
}

// ============================================================
// UART init
// ============================================================
static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = RX_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RX_UART, 4096, 0, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(RX_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RX_UART, RX_UART_TX_PIN, RX_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ============================================================
// WIFI
// ============================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi init done, connecting...");
}

// ============================================================
// MQTT EVENT HANDLER
// ============================================================
static void mqtt_publish_status(const char *msg)
{
    if (mqtt_client && mqtt_connected)
    {
        esp_mqtt_client_publish(mqtt_client, "esp32/status", msg, 0, 0, 0);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            mqtt_connected = true;

            esp_mqtt_client_subscribe(mqtt_client, TOPIC_CMD, 0);
            mqtt_publish_status("mqtt_online");
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            mqtt_connected = false;
            break;

        case MQTT_EVENT_DATA:
        {
            // Copy topic and payload into null-terminated strings
            char topic[128] = {0};
            char data[128]  = {0};

            int tlen = event->topic_len < (int)(sizeof(topic)-1) ? event->topic_len : (int)(sizeof(topic)-1);
            int dlen = event->data_len  < (int)(sizeof(data)-1)  ? event->data_len  : (int)(sizeof(data)-1);

            memcpy(topic, event->topic, tlen);
            memcpy(data,  event->data,  dlen);

            ESP_LOGI(TAG, "MQTT RX topic=%s data=%s", topic, data);

            if (strcmp(topic, TOPIC_CMD) == 0)
            {
                // PC sends: TEMP
                if (strcasecmp(data, "TEMP") == 0)
                {
                    stream_enabled = true;
                    mqtt_publish_status("stream_enabled");
                }
                else if (strcasecmp(data, "STOP") == 0)
                {
                    stream_enabled = false;
                    mqtt_publish_status("stream_disabled");
                }
            }
            break;
        }

        default:
            break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// ============================================================
// MQTT TELEMETRY PUBLISH TASK
// ============================================================
static void mqtt_publish_task(void *arg)
{
    char payload[256];

    while (1)
    {
        if (mqtt_connected && stream_enabled && latest.valid)
        {
            float temp_c = (float)latest.temp_x100 / 100.0f;

            // JSON payload (easy to parse on PC/app)
            snprintf(payload, sizeof(payload),
                     "{\"seq\":%u,\"ts\":%lu,\"adc\":%u,\"temp\":%.2f,\"flags\":%u}",
                     (unsigned)latest.seq,
                     (unsigned long)latest.ts,
                     (unsigned)latest.raw_adc,
                     temp_c,
                     (unsigned)latest.flags);

            esp_mqtt_client_publish(mqtt_client, TOPIC_TELEM, payload, 0, 0, 0);
        }

        // publish rate (10Hz)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================
// MAIN
// ============================================================
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    uart_init();
    ESP_LOGI(TAG, "UART%d RX=%d @ %d baud",
             RX_UART, (int)RX_UART_RX_PIN, RX_UART_BAUD);

    wifi_init_sta();

    // Start UART task immediately (safe)
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);

    // Wait for WiFi connect before starting MQTT
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    mqtt_start();

    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, NULL, 8, NULL);

    // IMPORTANT: never return from app_main
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#include "wifi_manager.h"
#include "shared_data.h"
#include "motor_control.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mdns.h"

#define WIFI_SSID      "SS_NEW"
#define WIFI_PASS      "qwertyuiop"
#define MDNS_HOSTNAME  "project"
#define WIFI_MAX_RETRY 5
#define WIFI_TIMEOUT_MS 15000

static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT      = BIT1;

static int retry_count = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        printf("WiFi disconnected, reason: %d\n", event->reason);

        auto_mode = 0;
        manual_override = 0;
        motor_all_stop();
        motor_fr_speed = 0;
        motor_fl_speed = 0;
        motor_br_speed = 0;
        motor_bl_speed = 0;
        printf("WiFi lost -> robot stopped\n");

        if (retry_count < WIFI_MAX_RETRY) {
            retry_count++;
            printf("Reconnecting... attempt %d/%d\n", retry_count, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        printf("Got IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void start_mdns(void)
{
    esp_err_t err = mdns_init();
    if (err) {
        printf("mDNS init failed: %s\n", esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set("ESP32 Robot Dashboard");

    mdns_txt_item_t service_txt[] = {
        {"board", "esp32"},
        {"path", "/"}
    };
    mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, service_txt, 2);
    printf("mDNS started: %s.local\n", MDNS_HOSTNAME);
}

void wifi_manager_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = { 0 };
    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    printf("Connecting to WiFi: %s\n", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        printf("WiFi connected!\n");
        start_mdns();
    } else if (bits & WIFI_FAIL_BIT) {
        printf("WiFi failed after %d retries\n", WIFI_MAX_RETRY);
    } else {
        printf("WiFi connection timeout\n");
    }
}

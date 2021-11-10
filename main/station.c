/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "station.h"
#include "station_config.h"
#include "ftp_server.h"
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;
bool wifi_connect = false;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        wifi_connect = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{

    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    //tcpip_adapter_set_ip_info(tcpip_adapter_if_t tcpip_if, tcpip_adapter_ip_info_t *ip_info)
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD
        },
    };
    strncpy((void *)wifi_config.sta.ssid, connect_userstr, sizeof(wifi_config.sta.ssid));
    strncpy((void *)wifi_config.sta.password, connect_passwordstr, sizeof(wifi_config.sta.password));
    /* Setting a password implies station will connect to all security modes including WEP/WPA.
        * However these modes are deprecated and not advisable to be used. Incase your Access point
        * doesn't support WPA2, these mode can be enabled by commenting below line */

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    // ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    // ESP_ERROR_CHECK(esp_wifi_start() );

    tcpip_adapter_ip_info_t static_ip_info;
    IP4_ADDR(&static_ip_info.ip, connect_ip[0], connect_ip[1], connect_ip[2] ,connect_ip[3]);
    IP4_ADDR(&static_ip_info.gw, connect_gw[0], connect_gw[1], connect_gw[2], connect_gw[3]);
    IP4_ADDR(&static_ip_info.netmask, connect_netmask[0], connect_netmask[1], connect_netmask[2], connect_netmask[3]);
    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);
    vTaskDelay(50);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &static_ip_info);
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    //sdk_wifi_station_connect();
    //_is_esp_connected_to_wifi();



    // tcpip_adapter_ip_info_t info;
    // struct softap_config cfg;
    // wifi_softap_get_config(&cfg);
    // strcpy((char *)cfg.ssid, "ESP8266");
    // cfg.ssid_len = strlen((char*)cfg.ssid);
    // wifi_softap_set_config_current(&cfg);
    // wifi_set_opmode(SOFTAP_MODE);

    // wifi_softap_dhcps_stop();
    // IP4_ADDR(&info.ip, 192, 168, 88, 1);
    // IP4_ADDR(&info.gw, 192, 168, 88, 1);
    // IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    // tcpip_adapter_set_ip_info(SOFTAP_IF, &info);
    // dhcps_lease_test();
    // wifi_softap_dhcps_start();

    ESP_LOGI(TAG, "wifi_init_sta finished.");






    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

// void _is_esp_connected_to_wifi(void){
// int status = sdk_wifi_station_get_connect_status();

// while(status != STATION_GOT_IP){
// 	vTaskDelay(100);
// 	status = sdk_wifi_station_get_connect_status();
// // TRACE("Connection Status: %d -- %s", status, sdk_wifi_status_2_string(status));
// }
// return;
// }
// void _station_init(void){

// char ssid[] = "WCS";
// char password[] = "RTSR&DWCS";
// struct sdk_station_config _station_info;
// struct ip_info static_ip_info = {0};

// TRACE("SSID:%s, Password: %s", ssid, password);
// strcpy((char *)_station_info.ssid, ssid);
// strcpy((char *)_station_info.password, password);
// _station_info.bssid_set = 0;

// //Delete all existing wifi stuff in the EEPROM
// // sdk_wifi_station_disconnect();
// // sdk_wifi_softap_stop();
// // sdk_wifi_station_stop();
// // sdk_wifi_set_opmode(NULL_MODE);
// // vTaskDelay(500);

// TRACE("dhcp status : %d", sdk_wifi_station_dhcpc_status());
// IP4_ADDR(&static_ip_info.ip, 192, 168, 5 ,133);
// IP4_ADDR(&static_ip_info.gw, 192, 168, 5, 1);
// IP4_ADDR(&static_ip_info.netmask, 255, 255, 255, 0);
// sdk_wifi_station_dhcpc_stop();
// vTaskDelay(50);
// //Must call sdk_wifi_set_opmode before sdk_wifi_station_set_config
// sdk_wifi_set_opmode(STATION_MODE);
// TRACE("static ip set status : %d", sdk_wifi_set_ip_info(STATION_IF, &static_ip_info));
// sdk_wifi_station_set_config(&_station_info);
// // sdk_wifi_station_start();
// sdk_wifi_station_connect();
// _is_esp_connected_to_wifi();
// TRACE("ESP Connected to WiFi %s", ssid);
// }
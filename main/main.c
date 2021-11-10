/* Hello World Example

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

#include "esp_spi_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/gpio.h"

// #include "lwip/ip4_addr.h"
// #include "lwip/sockets.h"
// #include "lwip/dns.h"
// #include "lwip/netdb.h"

#include "station.h"
#include "http_server.h"
#include "ftp_server.h"

#include "driver/uart.h"
#include "uart_events.h"

QueueHandle_t test_queue;
extern QueueHandle_t uart_queue;
extern QueueHandle_t ftp_queue;
extern uint8_t buff_to_usart[];
#define SEND_STRING "ESP SEND"
#define RECEIVE_STRING "ESP RECEIVE"

void ICACHE_FLASH_ATTR LEDBlinkTask(void *pvParameters)
{
    //uart_event_t ftp_event;

    //test_queue = xQueueCreate(4, sizeof(uart_event_t));
    while (1)
    {
        // Delay and turn on
        
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(500 / portTICK_RATE_MS);

        // Delay and LED off
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void led_init()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = GPIO_Pin_2;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void app_main(void)
{
    BaseType_t xReturned;
    ftp_queue = xQueueCreate(40, sizeof(serial_event_t));
    led_init();
    uart_events_init(); 
    //uart_init(500000);

    //ESP_ERROR_CHECK(nvs_flash_init());
    //wifi_init_sta();
  
    //start_tests();

    // struct ip_info info;
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
    // wifi_set_ip_info(SOFTAP_IF, &info);
    // dhcps_lease_test();
    // wifi_softap_dhcps_start();
    //
    // This task blinks the LED continuously
    
    xReturned = xTaskCreate(vBasicFTPServer, (const char *)"FTPServer", 1024*14, NULL, 2, NULL);
    while(xReturned != pdPASS){
        ESP_LOGI("START","ERROR!!! Error create task FTPServer");
        //send_log_str("FTP Server start!\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    //xTaskCreate(ServerTask, (signed char *)"Server", 256, NULL, 2, NULL);
    /* Start the FTP server: FAT + SDcard */
    //xTaskCreate(vBasicFTPServer, (signed char *)"FTP", 1024, NULL, 2, NULL);
    //(void)sys_thread_new("FTP", vBasicFTPServer, NULL, 1024, FTP_TASK_PRIORITY);
    /*Serial configuration: UART & SPI*/
    //xTaskCreate(vSerialBridgeConfiguration, (const signed portCHAR *)"CONF", SRL_BRIDGE_BUFFER_LIMIT, NULL, SERIAL_BRIDGE_TASK_PRIORITY, NULL);
}

	///!!!!!!!!!!!!!!!!!!!!!
	///!!!!!!!!!!!!!!!!!!!!!
	///!!!!!!!!!!!!!!!!!!!!!
	///!!!!!!!!!!!!!!!!!!!!!
	//Установить CONFIG_LWIP_TCP_QUEUE_OOSEQ 0
	///!!!!!!!!!!!!!!!!!!!!!
	///!!!!!!!!!!!!!!!!!!!!!
	///!!!!!!!!!!!!!!!!!!!!!
	///!!!!!!!!!!!!!!!!!!!!!
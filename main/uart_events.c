// Copyright 2018-2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "uart_events.h"
#include "ftp_server.h"

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: on
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

QueueHandle_t uart_queue;
extern QueueHandle_t ftp_queue;
extern QueueHandle_t test_queue;
SemaphoreHandle_t USERT_Mutex;

uint8_t buff_to_usart[USART_BUFFER_SIZE];
uint8_t buff_from_usart1[USART_BUFFER_SIZE];
uint8_t buff_from_usart2[USART_BUFFER_SIZE];
bool num_buff = false;
bool large_packet = false;
int len_large_packet = 0;
static void uart_event_task(void *pvParameters)
{
    uart_event_t uart_event;
    serial_event_t serial_event;
    int length = 0;

    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&uart_event, (portTickType)portMAX_DELAY))
        {

            //ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);

            switch (uart_event.type)
            {
            // Event of UART receving data
            // We'd better handler data event fast, there would be much more data events than
            // other types of events. If we take too much time on data event, the queue might be full.
            case UART_DATA:
                //ESP_LOGI(TAG, "[UART RECEIVE EVT]: %d", uart_event.size);
                ESP_ERROR_CHECK(uart_get_buffered_data_len(EX_UART_NUM, (size_t *)&length));
                length = uart_event.size;
                uint8_t *buff;
                if ((length == 120) && (!large_packet))
                {
                    large_packet = true;
                }

                COM_Disable_Receive();

                if (!large_packet)
                {
                    if (num_buff)
                    {
                        buff = buff_from_usart1;
                        if (length < sizeof(buff_from_usart1))
                            buff_from_usart1[length] = 0;
                    }
                    else
                    {
                        buff = buff_from_usart2;
                        if (length < sizeof(buff_from_usart2))
                            buff_from_usart2[length] = 0;
                    }
                    
                    // ESP_LOGI(TAG, "[DATA EVT]:");
                    // uart_write_bytes(EX_UART_NUM, (const char *) buff_from_usart, uart_event.size);
                    uart_read_bytes(EX_UART_NUM, buff, length, portMAX_DELAY);
                    serial_event.size = length;
                    serial_event.addr = buff;
                    // if(uxQueueMessagesWaiting(ftp_queue) > 0) {

                    // }
                    large_packet = false;
                    len_large_packet = 0;
                    num_buff = !num_buff;
                    if (pdFALSE == xQueueSend(ftp_queue, (void *)&serial_event, (portTickType)portMAX_DELAY))
                    {
                        //ESP_LOGI(TAG, "FTP event queue full");
                    }
                }
                else
                {
                    if(!num_buff)
                        buff = (buff_from_usart1 + len_large_packet);
                    else
                        buff = (buff_from_usart2 + len_large_packet);
                    if (length != 1)
                        len_large_packet += length;
                    if (length != 120)
                    {
                        if(length == 1){
                            uint8_t ch = 2;
                            uart_read_bytes(EX_UART_NUM, &ch, 1, portMAX_DELAY);
                        }
                        else{
                            uart_read_bytes(EX_UART_NUM, buff, length, portMAX_DELAY);
                        }

                        
                        serial_event.size = len_large_packet;
                        if(!num_buff)
                            serial_event.addr = buff_from_usart1;
                        else
                            serial_event.addr = buff_from_usart2;
                        large_packet = false;
                        len_large_packet = 0;
                        num_buff = !num_buff;
                        if (pdFALSE == xQueueSend(ftp_queue, (void *)&serial_event, (portTickType)portMAX_DELAY))
                        {
                            //ESP_LOGI(TAG, "FTP event queue full");
                        }


                    } else {
                        uart_read_bytes(EX_UART_NUM, buff, length, portMAX_DELAY);
                    }
                }

                break;

            //case UART_DATA_TO_TRANSMIT:
            //ESP_LOGI(TAG, "[UART TRANSMIT EVT]: %d", uart_event.size);
            //uart_write_bytes(EX_UART_NUM, (const char *)buff_to_usart, uart_event.size);
            //USART_MUTEX_EXIT;
            //break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                //ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                send_log_str("\n\n\nERROR!!!!! UART_FIFO_OVF!!!!\n\n\n");
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart_queue);
                break;

            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                //ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.

                send_log_str("\n\n\nERROR!!!!! UART_BUFFER_FULL!!!!\n\n\n");

                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart_queue);
                break;

            case UART_PARITY_ERR:
                send_log_str("\n\n\nERROR!!!!! UART_PARITY_ERR!!!!\n\n\n");
                //ESP_LOGI(TAG, "uart parity error");
                break;

            // Event of UART frame error
            case UART_FRAME_ERR:
                send_log_str("\n\n\nERROR!!!!! UART_FRAME_ERR!!!!\n\n\n");
                //ESP_LOGI(TAG, "uart frame error");
                break;

            // Others
            default:
                send_log_sprintf("\n\n\nERROR!!!!! uart event type: %d!!!!\n\n\n", uart_event.type);
                //ESP_LOGI(TAG, "uart event type: %d", uart_event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void com_gpio_init()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = COM_Output_Pin_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = COM_Input_Pin_SEL;
    gpio_config(&io_conf);
}

void COM_Enable_Receive(void)
{
    USART_MUTEX_ENTER;
    gpio_set_level(COM_Output_Pin, 1);
    USART_MUTEX_EXIT;
}

void COM_Disable_Receive(void)
{
    USART_MUTEX_ENTER;
    gpio_set_level(COM_Output_Pin, 0);
    USART_MUTEX_EXIT;
}

void uart_events_init(void)
{
    com_gpio_init();
    // Configure parameters of an UART driver,
    // communication pins and install the driver
    uart_config_t uart_config = {
        //.baud_rate = 1152000,
        //.baud_rate = 74880,
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(EX_UART_NUM, &uart_config);

    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, USART_BUFFER_SIZE * 2, USART_BUFFER_SIZE * 2, 100, &uart_queue, 0);
    //uart_set_rx_timeout(EX_UART_NUM, 10);
    vSemaphoreCreateBinary(USERT_Mutex);
    COM_Enable_Receive();
    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048 * 3, NULL, 2, NULL);
}

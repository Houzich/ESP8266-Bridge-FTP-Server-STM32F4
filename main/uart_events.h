/*

 */

#ifndef _UART_EVENTS_H_
#define _UART_EVENTS_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define EX_UART_NUM 0

extern void uart_events_init(void);

extern SemaphoreHandle_t USERT_Mutex;
#define USART_MUTEX_ENTER xSemaphoreTake(USERT_Mutex, portMAX_DELAY)
#define USART_MUTEX_EXIT xSemaphoreGive(USERT_Mutex)

 //D6
#define COM_Input_Pin  12
#define COM_Input_Pin_SEL  (1ULL<<COM_Input_Pin)
 //D5
#define COM_Output_Pin  14
#define COM_Output_Pin_SEL  (1ULL<<COM_Output_Pin)

/**
 * @brief Event structure used in UART event queue
 */
typedef struct {
    size_t size;            /*!< UART data size for UART_DATA event*/
    uint8_t *addr;
} serial_event_t;

void COM_Enable_Receive(void);
void COM_Disable_Receive(void);

#endif /* _UART_EVENTS_H_ */

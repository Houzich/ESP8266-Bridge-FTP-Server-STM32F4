#include <string.h>

/* ------------------------ FreeRTOS includes ----------------------------- */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ------------------------ lwIP includes --------------------------------- */
#include "lwip/api.h"

/* ------------------------ Project includes ------------------------------ */
//#include "SD.h"         /* SD Card Driver (SPI mode) */
//#include "FAT.h"

/* ------------------------ USART includes ------------------------------ */
#include "driver/uart.h"
#include "driver/gpio.h"
#include "uart_events.h"

/* ------------------------ Project includes ------------------------------ */
#include "ftp_server.h"
#include "utilities.h"
extern void wifi_init_sta(void);
extern bool wifi_connect;

#define FTP_USERNAME "user"
#define FTP_PASSWORD "123"

/*Handle SD card*/
//static FATHandler *SD_FTP_Handle;

QueueHandle_t ftp_queue;
extern QueueHandle_t uart_queue;
extern uint8_t buff_to_usart[];
extern uint8_t buff_from_usart1[];
extern uint8_t buff_from_usart2[];
uint8_t ftp_buff_to_usart[USART_BUFFER_SIZE];
uint8_t ftp_buff_from_usart[USART_BUFFER_SIZE];

	char connect_ipstr[50] = {0};
	char connect_gwstr[50] = {0};
	char connect_netmaskstr[50] = {0};
	char connect_userstr[50] = {0};
	char connect_passwordstr[50] = {0};
	uint8_t connect_ip[4] = {0};
	uint8_t connect_gw[4] = {0};
	uint8_t connect_netmask[4] = {0};

extern void ICACHE_FLASH_ATTR LEDBlinkTask(void *pvParameters);

#define packet_receive ((ESPPacket_Typedef *)ftp_buff_from_usart)
#define packet_send ((ESPPacket_Typedef *)ftp_buff_to_usart)

uint8_t buff_ftp[FTP_BUFFERS_SIZE];

///////SYST
/**
 * FTP Server Main Control Socket Parser: requests and responses
 *  Available requests: USER, PASS, PORT, QUIT
 *
 * @param connection descriptor
 * @param buffer to hold FTP requests
 * @return none
 */
static uint8_t
vFTPConnection(struct netconn *connfd);

/**
 * FTP Server Main Data Socket Parser: requests and responses
 *  Available requests: NLST, RTR, STOR
 *
 * @param connection descriptor
 * @param string containing data socket port number and address
 * @return none
 */

static void send_to_uart(uint8_t *buff, size_t len)
{
  uart_event_t usart_event;
  while (gpio_get_level(COM_Input_Pin) == 0)
    vTaskDelay(1 / portTICK_RATE_MS);
  uart_wait_tx_done(EX_UART_NUM, portMAX_DELAY); 
  //USART_MUTEX_ENTER;
  if (memcpy((void *)buff_to_usart, (void *)buff, len) == NULL)
  {
    return;
  }

  //usart_event.type = UART_DATA_TO_TRANSMIT;
  usart_event.size = len;
  // if (pdFALSE == xQueueSend(uart_queue, (void *)&usart_event, (portTickType)portMAX_DELAY))
  // {
  //   return;
  // }

  uart_write_bytes(EX_UART_NUM, (const char *)buff_to_usart, usart_event.size);
}

static void send_cmd(char *cmd)
{
  bzero(packet_send, sizeof(ESPPacket_Typedef));
  strcpy((void *)packet_send->data, cmd);
  size_t length_data = strlen(cmd);
  packet_send->header.id = ESP_FTP_COMMAND_ID;
  packet_send->header.len_data = length_data;
  size_t length = length_data + sizeof(ESPHeadr_Typedef);
  send_to_uart((void *)packet_send, length);
}
static void send_cmd_buff(uint8_t *cmd, size_t length_data)
{
  memcpy((void *)packet_send->data, cmd, length_data);
  packet_send->header.id = ESP_FTP_COMMAND_ID;
  packet_send->header.len_data = length_data;
  size_t length = length_data + sizeof(ESPHeadr_Typedef);
  send_to_uart((void *)packet_send, length);
}
static void send_data(uint8_t *data, size_t length_data)
{
  memcpy((void *)packet_send->data, data, length_data);
  packet_send->header.id = ESP_FTP_DATA_ID;
  packet_send->header.len_data = length_data;
  size_t length = length_data + sizeof(ESPHeadr_Typedef);
  send_to_uart((void *)packet_send, length);
}

void send_log_str(char *str)
{
  bzero(packet_send, sizeof(ESPPacket_Typedef));
  strcpy((void *)packet_send->data, str);
  size_t length_data = strlen(str);
  packet_send->header.id = ESP_LOG_ID;
  packet_send->header.len_data = length_data;
  size_t length = length_data + sizeof(ESPHeadr_Typedef);
  send_to_uart((void *)packet_send, length);
}

void send_log_sprintf(const char *format, ...)
{
  int length_data;
  va_list arg;
  va_start(arg, format);
  length_data = vsprintf((void *)packet_send->data, format, arg);
  va_end(arg);
  packet_send->header.id = ESP_LOG_ID;
  packet_send->header.len_data = length_data;
  size_t length = length_data + sizeof(ESPHeadr_Typedef);
  send_to_uart((void *)packet_send, length);
}

static BaseType_t receive_from_usart()
{
  serial_event_t event;
  BaseType_t ret = xQueueReceive(ftp_queue, (void *)&event, (portTickType)portMAX_DELAY);
  if (ret == true)
  {
    memcpy(ftp_buff_from_usart, event.addr, event.size);
    if (event.size != (packet_receive->header.len_data + sizeof(ESPHeadr_Typedef)))
    {
      send_log_sprintf("ERROR RECEIVE DATA!!! header.len_data: %d, event.size: %d", packet_receive->header.len_data, event.size);
      //vTaskDelay(1000 / portTICK_RATE_MS);
    }
    if (event.size < sizeof(ftp_buff_from_usart))
      ftp_buff_from_usart[event.size] = 0;
    COM_Enable_Receive();
  }

  return ret;
}

static BaseType_t receive_from_usart_delay(portTickType delay)
{
  serial_event_t event;
  BaseType_t ret = xQueueReceive(ftp_queue, (void *)&event, delay);
  if (ret == true)
  {
    memcpy(ftp_buff_from_usart, event.addr, event.size);
    if (event.size != (packet_receive->header.len_data + sizeof(ESPHeadr_Typedef)))
    {
      send_log_sprintf("ERROR RECEIVE DATA!!! header.len_data: %d, event.size: %d", packet_receive->header.len_data, event.size);
      //vTaskDelay(1000 / portTICK_RATE_MS);
    }
    if (event.size < sizeof(ftp_buff_from_usart))
      ftp_buff_from_usart[event.size] = 0;

    COM_Enable_Receive();
  }
  return ret;
}


/*FSL:sprintf prototype*/
//INT
//sprintf(char *, const char *, ... );

/********************Private Functions ***************************************/

/**
 * FTP Server Main Control Socket Parser: requests and responses
 *  Available requests: USER, PASS, PORT, QUIT
 *
 * @param connection descriptor
 * @param buffer to hold FTP requests
 * @return none
 */
static uint8_t ICACHE_FLASH_ATTR
vFTPConnection(struct netconn *connfd)
{
  struct netconn conndata;
  err_t err;
  send_cmd(ESP_OPEN_CMD);

  /*send FTP server first RESPONSE*/
  //****RESPONSE OK CONNECTED
  //netconn_write(connfd, FTP_WELCOME_RESPONSE, strlen(FTP_WELCOME_RESPONSE), 0);
  if (receive_from_usart())
  {
    if (FTP_Check_Reset_From_MCU(packet_receive->data))
    {
      //send_log_str("RESET FTP Server!!!!\n");
      goto FTP_TCP_EXIT_LOW;
    }
    netconn_write(connfd, (const void *)packet_receive->data, (size_t)packet_receive->header.len_data, 0);
  }
  //send_log_str("RESPONSE OK CONNECTED\n");
  //vTaskDelay(40 / portTICK_RATE_MS);
  netconn_set_timeout((void *)connfd, 1 /*timeout*/);
  do
  {
    /*if reception is OK: wait for REQUEST from client*/
    int32_t len = netconn_rcv_req((void *)connfd, buff_ftp, NULL, 0, &err);
    if ((err != ERR_OK) && (err != ERR_TIMEOUT))
    {
      /*session closed by client*/
      send_log_sprintf("Exit connfd error %d!!!!\n", err);
      send_cmd(ESP_ERROR_REQUEST);
      break;
    }

    if ((len != 0) && (err == ERR_OK))
    {
      send_cmd_buff(buff_ftp, len);
      //continue;
      //send_log_str("receive len 0!!!!\n");
    }
    if (len == -1)
    {
      /*session closed by client*/
      send_log_str("Session closed by clien!!!!\n");
      send_cmd(ESP_ERROR_REQUEST);
      break;
    }
    if (receive_from_usart_delay(1) == pdPASS)
    {
      if (packet_receive->header.id == ESP_FTP_COMMAND_ID)
      {
        if (FTP_Check_Reset_From_MCU(packet_receive->data))
        {
          send_log_str("RESET From MCU!!!!\n");
          break;
        }
        if (FTP_Check_Close_DataPort(packet_receive->data, true))
        {
            continue;
        }        
        if (FTP_Check_Open_DataPort(packet_receive->data, false))
        {
          if (FTP_DataState(connfd, &conndata, packet_receive->data) != ERR_OK)
          {
            send_log_str("Exit from DataState with error!!!!\n");
            break;
          }
          else
          {
            send_log_str("Exit from DataState successfully!\n");
            continue;
          }
        }

        netconn_write(connfd, (const void *)packet_receive->data, (size_t)packet_receive->header.len_data, NETCONN_COPY);
      }
    }
    // err_t
    // netconn_write_partly(struct netconn *conn, const void *dataptr, size_t size,
    //                      u8_t apiflags, size_t *bytes_written)

  } while (1);
FTP_TCP_EXIT_LOW:
  /*client closing the session*/
  //netconn_close(connfd);
  //netconn_delete(connfd);

  /*close the session!!*/

  return 1; /*default close value*/
}

/**
 * Closes or Leave session depending on client request
 *
 * @param connection descriptor
 * @param buffer space 
 * @return 0 keep session, otherwise session needs to be closed
 */
uint8_t ICACHE_FLASH_ATTR
FTP_QUIT_OR_WRONG_REQUEST(struct netconn *connfd, uint8_t *alloc_rq)
{
  if (strstr((void *)alloc_rq, FTP_QUIT_REQUEST) != NULL)
  {
    //****RESPONSE CLOSING SESSION: BYE
    netconn_write(connfd, FTP_QUIT_RESPONSE, str_len(FTP_QUIT_RESPONSE), 0);
    return 1; /*close session*/
  }
  else
  {
    //****UNKNOWN REQUEST
    netconn_write((void *)connfd, FTP_UNKNOWN_RESPONSE, str_len(FTP_UNKNOWN_RESPONSE), 0);
    return 0; /*keep session*/
  }
}

bool ICACHE_FLASH_ATTR
FTP_Check_Reset_From_MCU(uint8_t *buff)
{
  //send_log_str("CHECK RESET FTP Server!!!!\n");
  if (strstr((void *)buff, ESP_RESET_CMD) != NULL)
  {
    return true; /*close session*/
  }
  return false; /*keep session*/
}
bool ICACHE_FLASH_ATTR
FTP_Check_Connect_From_MCU(uint8_t *buff)
{
  //send_log_str("CHECK RESET FTP Server!!!!\n");
  if (strstr((void *)buff, ESP_CONNECT_CMD) != NULL)
  {
    return true; /*close session*/
  }
  return false; /*keep session*/
}
bool ICACHE_FLASH_ATTR
FTP_Check_Close_DataPort(uint8_t *buff, bool send_resp)
{
  //send_log_str("CHECK RESET FTP Server!!!!\n");
  if (strstr((void *)buff, "CLOSE DATAPORT") != NULL)
  {
    if (send_resp)
      send_cmd(ESP_CLOSE_DATAPORT_OK_RESPONSE);
    return true; /*close session*/
  }
  return false; /*keep session*/
}

bool ICACHE_FLASH_ATTR
FTP_Check_Open_DataPort(uint8_t *buff, bool send_resp)
{
  if (strstr((void *)buff, "OPEN DATAPORT") != NULL)
  {
    if (send_resp)
      send_cmd(ESP_OPEN_DATAPORT_OK_RESPONSE);
    return true; /*close session*/
  }
  return false; /*keep session*/
}
/**
 * Open data socket: ftp server connects to client.
 *
 * @param ip address to connect to
 * @param tcp port to connect to
 * @return connection descriptor, if NULL error, other OK.
 */
struct netconn *ICACHE_FLASH_ATTR
FTP_OpenDataPort(ip_addr_t *addr, uint16_t port)
{
  /*START:open specific port requested by PORT*/
  /* Create a new TCP connection handle. */
  struct netconn *conn_data;

  /*create data port*/
  if ((conn_data = netconn_new(NETCONN_TCP)) == NULL)
  {

    return NULL; /*error*/
  }

  //ESP_LOGI("FTP_OpenDataPort", "netconn_new!\n");
  /*wait until it's linked to server*/
  if (netconn_connect(conn_data, (const ip_addr_t *)addr, port) != ERR_OK)
  {
    return NULL; /*error*/
  }

  //ESP_LOGI("FTP_OpenDataPort", "netconn_connect!\n");
  /*set timeout for this connection*/
  netconn_set_timeout((void *)conn_data, 4000 /*timeout*/);

  /*END*/
  return conn_data;
}

/**
 * Close data socket
 *
 * @param connection descriptor
 * @return 0 if connection was closed
 */
static uint8_t ICACHE_FLASH_ATTR
FTP_CloseDataPort(struct netconn *conn_data)
{
  /*delete TCP connection*/
  netconn_close(conn_data);
  netconn_delete(conn_data);

  return 0;
}

err_enum_t ICACHE_FLASH_ATTR
FTP_DataState(struct netconn *conn, struct netconn *conn_data, uint8_t *cmd)
{
  err_t err;
  ip_addr_t ip;
  uint8_t ipaddr[4];
  uint16_t port;
  if (FTP_Check_Open_DataPort(cmd, false) == false)
    return ERR_VAL;
  ipaddr[0] = cmd[14];
  ipaddr[1] = cmd[15];
  ipaddr[2] = cmd[16];
  ipaddr[3] = cmd[17];
  IP4_ADDR(&ip, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
  port = ((uint16_t)cmd[18] << 8 | cmd[19]);

  conn_data = FTP_OpenDataPort(&ip, port);
  if (conn_data == NULL)
  {
    send_log_str("Cannot open port!!!\n");
    send_cmd(ESP_ERROR_REQUEST);
    return ERR_CONN;
  }
  send_log_sprintf("DataPort IP: %d.%d.%d.%d, %d\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], port);
  send_cmd(ESP_OPEN_DATAPORT_OK_RESPONSE);

  do
  {
    if (receive_from_usart_delay(1) == pdPASS)
    {

      if (packet_receive->header.id == ESP_FTP_DATA_ID)
      {
        netconn_set_timeout((void *)conn_data, 4000 /*timeout*/);
        //send_log_sprintf("Send data %d bytes %s\n", packet_receive->header.len_data, (char *)packet_receive->data);
        err = netconn_write(conn_data, (const void *)packet_receive->data, (size_t)packet_receive->header.len_data, NETCONN_COPY);
        //send_log_sprintf("netconn_write ret %d\n", err);
        if (err != ERR_OK){
          send_log_str("ERROR netconn_write DataPort!!!\n");
          goto EXIT_ERR_RST;
        }
        err = netconn_err(conn);
        if (err != ERR_OK){
          send_log_sprintf("ERROR netconn_err %d!!!\n", err);
        }

      }
      else if (packet_receive->header.id == ESP_FTP_COMMAND_ID)
      {

        if (FTP_Check_Reset_From_MCU(packet_receive->data))
        {
          send_log_str("Reset from MCU\n");
          goto EXIT_ERR_RST;
        }

        if (FTP_Check_Close_DataPort(packet_receive->data, true))
        {
          send_log_str("Close DataPort Command\n");
          break;
        }
        send_log_sprintf("In DataState Send command %d bytes: %s\n", packet_receive->header.len_data, (char *)packet_receive->data);
        netconn_set_timeout((void *)conn, 4000 /*timeout*/);
        err = netconn_write(conn, (const void *)packet_receive->data, (size_t)packet_receive->header.len_data, NETCONN_COPY);
        if (err != ERR_OK){
          send_log_str("ERROR netconn_write Command!!!\n");
          goto EXIT_ERR_RST;
        }
      }
    }
    netconn_set_timeout((void *)conn_data, 1 /*timeout*/);
    int len = netconn_rcv_req((void *)conn_data, buff_ftp, NULL, 0, &err);
    if ((err != ERR_OK) && (err != ERR_TIMEOUT))
    {
      /*session closed by client*/
      
      send_cmd(ESP_CLOSE_DATAPORT_CMD);
      if((err == ERR_ABRT)||(err == ERR_RST)||(err == ERR_CLSD)){
        send_log_sprintf("Session closed by client DATAPORT OK. Code %d\n", err);
        break;
      }else{
        send_log_sprintf("Session closed by client DATAPORT with Error %d\n", err);
        goto EXIT_ERR_RST;
      }
      
    }
    if ((len != 0) && (err == ERR_OK))
    {
      //send_log_sprintf("Send data DATAPORT len %d\n", len);
      send_data(buff_ftp, len);
    }
    if (len == -1)
    {
      send_log_sprintf("Session closed by client DATAPORT. FIN! \n");
      send_cmd(ESP_CLOSE_DATAPORT_CMD);
      break;
    }
    netconn_set_timeout((void *)conn, 1 /*timeout*/);
    len = netconn_rcv_req((void *)conn, buff_ftp, NULL, 0, &err);
    if ((err != ERR_OK) && (err != ERR_TIMEOUT))
    {
      /*session closed by client*/
      send_log_str("Session closed by client COMMANDPORT\n");
      send_cmd(ESP_ERROR_REQUEST);
      goto EXIT_ERR_RST;
    }
    if ((len != 0) && (err == ERR_OK))
    {
      send_cmd_buff(buff_ftp, len);
    }
    if (len == -1) 
    {
      send_log_sprintf("Session closed by client COMMANDPORT. FIN! \n");
      send_cmd(ESP_ERROR_REQUEST);
      goto EXIT_ERR_RST;
    }
  } while (1);

  send_log_str("Closing DataPort\n");
  FTP_CloseDataPort(conn_data);
  return ERR_OK;
EXIT_ERR_RST:
  FTP_CloseDataPort(conn_data);
  return ERR_RST;
}

/**
 * Start an embedded FTP server Task: 1 client and 1 file per transfer
 *
 * @param paremeter pointer to use with task
 * @return none
 */
void ICACHE_FLASH_ATTR
vBasicFTPServer(void *pvParameters)
{
  /*Connection descriptors for FTP control port*/
  struct netconn *conn, *connection;
  err_t err;

  uint8_t i = FTP_CLOSED; /*keep session information*/

  /* Parameters are not used - suppress compiler error. */
  (void)pvParameters;

  /*Apps buffer*/
  // SDbuffer = (uint8_t *)malloc(BLOCK_SIZE);
  // alloc_rq = (uint8_t *)malloc(FTP_REQUEST_SPACE);
  // if ((SDbuffer == NULL) ||
  //     (alloc_rq == NULL)
  // {
  //   free(SDbuffer);
  //   free(alloc_rq);
  //   send_log_str("FTPServerTask Error: malloc\nExit from Task\n");
  //   /*Task no longer needed, delete it!*/
  //   vTaskDelete(NULL);
  // }
  /* SD Card Initialization */
  //    if( (SD_FTP_Handle = FAT_INIT(SDbuffer)) == NULL )
  //    {
  //      /*delete requested memory*/
  //      mem_free(SDbuffer);
  //      mem_free(alloc_rq);
  //      /*error*/
  //      FAT_Close();
  //      /*Task no longer needed, delete it!*/
  //      vTaskDelete( NULL );
  //    }

  /* SD Card Initialization */

  /**********************FSL: socket start-up*******************************/
  while(1){
    if (receive_from_usart())
    {
      if (packet_receive->header.id == ESP_FTP_COMMAND_ID)
      {
        if(FTP_Check_Connect_From_MCU(packet_receive->data))
        {
          	 char *ptr = (void *)&packet_receive->data[sizeof(ESP_CONNECT_CMD) - 1];
             for(int i = 0; i < 300; i++){
							 if((ptr[i] == '\n')||(ptr[i] == '\r'))ptr[i] ='\0';
						 }
						 memset(connect_ipstr,0,sizeof(connect_ipstr));
						 memset(connect_gwstr,0,sizeof(connect_gwstr));						 
						 memset(connect_netmaskstr,0,sizeof(connect_netmaskstr));						 
						 memset(connect_userstr,0,sizeof(connect_userstr));						 
						 memset(connect_passwordstr,0,sizeof(connect_passwordstr));						 
						 
						 strncpy(connect_ipstr, ptr, sizeof(connect_ipstr) - 1);
						 ptr += strlen(connect_ipstr) + 2;
						 
						 strncpy(connect_gwstr, ptr, sizeof(connect_gwstr) - 1);
						 ptr += strlen(connect_gwstr) + 2;

						 strncpy(connect_netmaskstr, ptr, sizeof(connect_netmaskstr) - 1);
						 ptr += strlen(connect_netmaskstr) + 2;

						 strncpy(connect_userstr, ptr, sizeof(connect_userstr) - 1);
						 ptr += strlen(connect_userstr) + 2;

						 strncpy(connect_passwordstr, ptr, sizeof(connect_passwordstr) - 1);
						 ptr += strlen(connect_passwordstr) + 2;

						ip_convert_address(connect_ipstr, (void *)connect_ip);
						ip_convert_address(connect_gwstr, (void *)connect_gw);
						ip_convert_address(connect_netmaskstr, (void *)connect_netmask);
            wifi_init_sta();
            while(!wifi_connect) vTaskDelay(100 / portTICK_RATE_MS);
            xTaskCreate(LEDBlinkTask, (const char *)"Blink", 512, NULL, -2, NULL);
            break;
        }
      }
    }
  }
  /* Create a new TCP connection handle. */
  conn = netconn_new(NETCONN_TCP);
  if (conn == NULL)
  {
    send_log_str("FTPServerTask Error: Function netconn_new\n");
    asm("break 1,1");
    //continue;
  }
  /* Bind the connection to port 21 on any local IP address. */
  err = netconn_bind(conn, NULL, FTP_TCP_CONTROL_PORT);
  if (err != ERR_OK)
  {
    send_log_str("FTPServerTask Error: Function netconn_bind\n");
    asm("break 1,1");
    //continue;
  }
  /* Put the connection into LISTEN state. */
  err = netconn_listen(conn);
  if (err != ERR_OK)
  {
    send_log_str("FTPServerTask Error: Function netconn_listen\n");
    asm("break 1,1");
    //continue;
  }
  /*set timeout for this connection*/
  netconn_set_timeout((void *)conn, 1000 /*timeout*/);

 
  // for(;;){
  //   ESP_LOGI("START","connect to the AP fail");
  //   //send_log_str("FTP Server start!\n");
  //   vTaskDelay(1000 / portTICK_RATE_MS);
  // }
  for (;;) /*infinite loop*/
  {
    if (i == FTP_CLOSED) /*FTP_CLOSE*/
    {
      err = netconn_accept(conn, &connection);
      if (err == ERR_OK)
      {
        i = FTP_OPEN;
        /*set timeout for this connection*/
        netconn_set_timeout((void *)connection, 0 /*timeout*/);
        //send_log_str("FTP connect open!\n");
      }
      else
      {
        //принимаем с usart чтоб не забивать очередь
        if (receive_from_usart_delay(0))
        {
          if (FTP_Check_Reset_From_MCU(packet_receive->data))
          {
            //send_log_str("RESET FTP Server!!!!\n");
          }
        }
        //send_log_str("netconn_accept ERROR %d\n", err);
      }
    }
    else /*FTP_OPEN*/
    {
      send_log_str("FTP Server start!\n");
      /* Service connection */
      if (vFTPConnection(connection))
      {
        i = FTP_CLOSED;
        netconn_set_timeout((void *)connection, 1000 /*timeout*/);
      }
    }
  }

  /*never get here!*/
  return;
}

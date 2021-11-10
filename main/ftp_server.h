#ifndef _FTP_SERVER_H_
#define _FTP_SERVER_H_

#include "lwip/api.h"

enum
{
  FTP_CLOSED,
  FTP_OPEN
};

#define FTP_TASK_PRIORITY          tskIDLE_PRIORITY + 3

#define FTP_TCP_CONTROL_PORT       21

#define FTP_REQUEST_SPACE          500/*Client request should not be longer*/

/*FTP Server Requests*/
#define FTP_USER_REQUEST           "USER"
#define FTP_PASS_REQUEST           "PASS"              
#define FTP_QUIT_REQUEST           "QUIT"
#define FTP_PORT_REQUEST           "PORT"
#define FTP_NLST_REQUEST           "NLST"
#define FTP_STOR_REQUEST           "STOR"
#define FTP_RETR_REQUEST           "RETR"
#define FTP_DELE_REQUEST           "DELE"
#define FTP_SYST_REQUEST           "SYST"

#define ESP_ERROR_REQUEST					        "session closed by client"
#define ESP_OPEN_DATAPORT_CMD						"OPEN DATAPORT %c%c%c%c%c%c"
#define ESP_CLOSE_DATAPORT_CMD						"CLOSE DATAPORT"
#define ESP_OPEN_DATAPORT_OK_RESPONSE				"OPEN DATAPORT OK"
#define ESP_CLOSE_DATAPORT_OK_RESPONSE				"CLOSE DATAPORT OK"
#define ESP_RESET_CMD												"RESET FROM STM32"
#define ESP_OPEN_CMD												"OPEN"
#define ESP_CONNECT_CMD											"CONNECT "
/*FTP Server Response*/
#define FTP_WELCOME_RESPONSE        "220 Service Ready\r\n"
#define FTP_USER_RESPONSE           "331 USER OK. PASS needed\r\n"
#define FTP_PASS_FAIL_RESPONSE      "530 NOT LOGGUED IN\r\n"
#define FTP_PASS_OK_RESPONSE        "230 USR LOGGUED IN\r\n"
#define FTP_PORT_OK_RESPONSE        "200 PORT OK\r\n"
#define FTP_NLST_OK_RESPONSE        "150 NLST OK\r\n"
#define FTP_RETR_OK_RESPONSE        "150 RETR OK\r\n"
#define FTP_STOR_OK_RESPONSE        "150 STOR OK\r\n"
#define FTP_DELE_OK_RESPONSE        "150 DELE OK\r\n"
#define FTP_QUIT_RESPONSE           "221 BYE OK\r\n"
#define FTP_TRANSFER_OK_RESPONSE    "226 Transfer OK\r\n"
#define FTP_WRITE_FAIL_RESPONSE     "550 File unavailable\r\n"
#define FTP_CMD_NOT_IMP_RESPONSE    "502 Command Unimplemented\r\n"
#define FTP_DATA_PORT_FAILED        "425 Cannot open Data Port\r\n"
#define FTP_UNKNOWN_RESPONSE        "500 Unrecognized Command\r\n"
#define FTP_BAD_SEQUENCE_RESPONSE   "503 Bad Sequence of Commands\r\n"
#define FTP_SYST_OK_RESPONSE        "215 UNIX emulated by XXX.\r\n"

/********Prototype Functions************************************/

/**
 * Closes or Leave session depending on client request
 *
 * @param connection descriptor
 * @param buffer space 
 * @return 0 keep session, otherwise session needs to be closed
 */
uint8_t
FTP_QUIT_OR_WRONG_REQUEST(struct netconn *connfd, uint8_t *alloc_rq);



/**
 * Look if a file already exist with the name
 *  No case sensitive
 *
 * @param file name to look for on FAT
 * @return 0 if file was found, otherwise not
 */
uint8_t
FTP_Does_File_Exist(uint8_t *data);

/**
 * Callback to send files name to ethernet
 *
 * @param descriptor to use for sending
 * @param filename string
 * @return always 1
 */
uint8_t
FTP_SD_send(void* var, uint8_t *fileName);

/**
 * Callback to check if file exists
 *
 * @param filename to compare
 * @param filename string
 * @return 0 if file found, otherwise zero
 */
// uint8_t
// FTP_CompareFile(void* var, uint8_t *fileName);

/**
 * Returns thru connection descriptor all files names in FAT
 *
 * @param connection descriptor to send files' names
 * @return 0 if read was OK, otherwise not
 */
uint8_t
FTP_Read_List_Of_Files(struct netconn *connfd);

/**
 * Check if filename exists in FAT system for reading
 *
 * @param file name to check
 * @return 0 if read is possible, otherwise not
 */
uint8_t
FTP_Read_Is_Possible(uint8_t *data);

/**
 * Gets a file from FAT and send it to a connection descriptor
 *  Call FTP_Read_Is_Possible(...)
 *
 * @param connection descriptor to write data read data
 * @return 0 if read was OK, otherwise not
 */
uint8_t
FTP_Read_File(struct netconn *connfd);

/**
 * Puts a file from a connection descriptor and send it to FAT
 *  First check if file do not exist to avoid duplication of files in FAT
 *
 * @param connection descriptor to use read data
 * @param file name to write.
 * @return 0 if read was OK, otherwise not
 */
uint8_t
FTP_Write_File(struct netconn *connfd, uint8_t *data);

/**
 * Start an embedded FTP server Task: 1 client and 1 file per transfer
 *
 * @param paremeter pointer to use with task
 * @return none
 */
void
vBasicFTPServer( void *pvParameters );

bool ICACHE_FLASH_ATTR
FTP_Check_Reset_From_MCU(uint8_t *buff);

err_enum_t ICACHE_FLASH_ATTR
FTP_DataState(struct netconn *conn, struct netconn *conn_data, uint8_t *cmd);

bool ICACHE_FLASH_ATTR
FTP_Check_Close_DataPort(uint8_t *buff, bool send_resp);

bool ICACHE_FLASH_ATTR
FTP_Check_Open_DataPort(uint8_t *buff, bool send_resp);

#define USART_BUFFER_SIZE  (CONFIG_LWIP_TCP_MSS * 1) + sizeof(ESPHeadr_Typedef)
#define FTP_BUFFERS_SIZE (CONFIG_LWIP_TCP_MSS * 1)
#define	ESP_FTP_COMMAND_ID	0x0200
#define	ESP_FTP_DATA_ID	0x0300
#define	ESP_LOG_ID	0x0100

#define member_size(type, member) sizeof(((type *)0)->member)

#pragma pack(push, 1)
typedef struct {
	uint16_t id;
	uint16_t len_data;
}ESPHeadr_Typedef;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	ESPHeadr_Typedef header;
	uint8_t data[USART_BUFFER_SIZE - sizeof(ESPHeadr_Typedef)];
}ESPPacket_Typedef;
#pragma pack(pop)

extern char connect_ipstr[50];
extern char connect_gwstr[50];
extern char connect_netmaskstr[50];
extern char connect_userstr[50];
extern char connect_passwordstr[50];
extern uint8_t connect_ip[4];
extern uint8_t connect_gw[4];
extern uint8_t connect_netmask[4];

void send_log_str(char *str);
void send_log_sprintf(const char *format, ...);

#endif
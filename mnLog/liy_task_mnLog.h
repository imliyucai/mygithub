/*
 *  liy_task_mnLog.h   
 *  header file of task file
 *  
 *  liyucai  2023-3-27
 *
 */

#ifndef __LIY_TASK_MNLOG_H	
#define __LIY_TASK_MNLOG_H

#ifdef __cplusplus
 extern "C" {
#endif

//////////////////////////////////////////////////////////////////67
//#include <stdint.h>
//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"




			
#define UARTBAUDRATE_9600   (9600)
#define UARTBAUDRATE_115200   (115200)

#define UART1_RX_DMA_BUFSIZE  128
#define UART1_TX_DMA_BUFSIZE  64


void UART1tohost_Init( uint32_t bdrate );

void UART1tohost_SendByte( uint8_t byte );
void UART1tohost_SendData( uint8_t* sp, uint16_t length );
void UART1tohost_SendString( char *sp );

void liy_Uart1_Test( void );


void liy_task_mnLog( void* p_arg );


//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67
#ifdef __cplusplus
}
#endif

#endif   /* __LIY_TASK_MNLOG_H */




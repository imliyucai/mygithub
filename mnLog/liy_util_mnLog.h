/*
 *  liy_util_mnLog.h   
 *  header file of mnLog utility functions
 *  
 *  liyucai  2023-4-3
 *
 */

#ifndef __LIY_UTIL_MNLOG_H	
#define __LIY_UTIL_MNLOG_H

#ifdef __cplusplus
 extern "C" {
#endif

//////////////////////////////////////////////////////////////////67
//#include <stdint.h>
//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"





//////////////////////////////////////////////////////////////////67
typedef struct
{
  uint32_t unixtime;
	uint16_t logcode;
	uint16_t logdata;
} mnlog_t;

#define MAGIC_ADDRESS_FM25    (0x40)
#define LOG_ADDRESS_LASTWRITE    (0x44)
#define LOG_ADDRESS_LASTREAD    (0x48)



//////////////////////////////////////////////////////////////////67
void mnLog_Init( void );
void mnLog_Format( void );   
void mnLog_WriteLog( uint16_t logcode, uint16_t logdata );

uint8_t mnLog_CheckNewLog( void );
void mnLog_ReadNewLog( mnlog_t* plog );
//////////////////////////////////////////////////////////////////67


#ifdef __cplusplus
}
#endif

#endif   /* __LIY_UTIL_MNLOG_H */




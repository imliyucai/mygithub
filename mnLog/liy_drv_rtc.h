/*
 *  liy_drv_rtc.h   
 *  header file of driver to RTC module in MCU STM32F4
 *  
 *  liyucai  2023-4-1
 *
 */

#ifndef __LIY_DRV_RTC_H	
#define __LIY_DRV_RTC_H

#ifdef __cplusplus
 extern "C" {
#endif

//////////////////////////////////////////////////////////////////67
//#include <stdint.h>
//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"


typedef struct
{
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t weekday;
}  timedate;

//////////////////////////////////////////////////////////////////67
void rtc_Config( void );

//uint8_t timedate[7];  // hour,minute,second, year,month,day, weekday
void rtc_SetTimeDate( timedate* ptimdat );
void rtc_GetTimeDate( timedate* ptimdat );

//////////////////////////////////////////////////////////////////67
void rtc_GettoUnixtime( uint32_t* punixtime );
void rtc_SetfromUnixtime( uint32_t unixtime );

//////////////////////////////////////////////////////////////////67
#ifdef __cplusplus
}
#endif

#endif   /* __LIY_DRV_RTC_H */




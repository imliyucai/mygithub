/*
 *  liy_drv_rtc.c  
 *     RTC(in STM32F4) driver.
 *     
 *    
 *
 *  liyucai  2023-4-1
 *
 */


//////////////////////////////////////////////////////////////////67

#include <stdint.h>
#include <string.h>   

#include <time.h>

//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"


#include  "..\liy_mnLog\liy_drv_rtc.h"

//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67


//////////////////////////////////////////////////////////////////67

void rtc_Config( void )
{
  RTC_InitTypeDef rtc_InitStruct;
	
  // Enable the PWR clock 
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE );

  // Allow access to RTC 
  PWR_BackupAccessCmd(ENABLE);
 
#define 	RTC_CLOCK_SOURCE_LSE
///////////////////////////////////////////////////////56	
#if defined (RTC_CLOCK_SOURCE_LSE)  
	// Enable the LSE OSC 
  RCC_LSEConfig( RCC_LSE_ON );

  /* Wait till LSE is ready */  
  while( RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET )
  { ; }

  // Select the RTC Clock Source 
  RCC_RTCCLKConfig( RCC_RTCCLKSource_LSE );
	
#else	
//#if defined (RTC_CLOCK_SOURCE_LSI)  
  // Enable the LSI OSC  
  RCC_LSICmd( ENABLE );

  // Wait till LSI is ready   
  while( RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET )
  { ; }

  // Select the RTC Clock Source 
  RCC_RTCCLKConfig( RCC_RTCCLKSource_LSI );
  
#endif 
  
  // Enable the RTC Clock 
  RCC_RTCCLKCmd( ENABLE );

  // Wait for RTC APB registers synchronisation 
  RTC_WaitForSynchro( );

  // Configure the RTC data register and RTC prescaler 
  rtc_InitStruct.RTC_AsynchPrediv = 0x7f;   // #define ASYHCHPREDIV 0X7F
  rtc_InitStruct.RTC_SynchPrediv = 0xff;    // #define SYHCHPREDIV 0XFF
  rtc_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
	 
  if( RTC_Init( &rtc_InitStruct ) == ERROR )
  {
    //printf("\n\r   ***** RTC Prescaler Config failed **** \n\r");
  }
}
//////////////////////////////////////////////////////////////////67

void rtc_SetTimeDate( timedate* ptimdat )
{   //uint8_t timedate[7];  // hour,minute,second, year,month,day, weekday
  RTC_TimeTypeDef rtc_TimeStruct;
  RTC_DateTypeDef rtc_DateStruct;
	
  rtc_TimeStruct.RTC_H12 = RTC_HourFormat_24;
  rtc_TimeStruct.RTC_Hours = ptimdat->hour;         //timedate[0];
  rtc_TimeStruct.RTC_Minutes = ptimdat->minute;     //timedate[1];
  rtc_TimeStruct.RTC_Seconds = ptimdat->second;     //timedate[2];
   RTC_SetTime( RTC_Format_BIN, &rtc_TimeStruct ); 
   RTC_WriteBackupRegister( RTC_BKP_DR0, 0x32f2 );   // #define RTC_SET_KEY	0xcd12
	
  rtc_DateStruct.RTC_WeekDay = ptimdat->weekday;   //timedate[6];
  rtc_DateStruct.RTC_Date = ptimdat->day;          //timedate[5];
  rtc_DateStruct.RTC_Month = ptimdat->month;       //timedate[4];
  rtc_DateStruct.RTC_Year = ptimdat->year;         //timedate[3];
   RTC_SetDate( RTC_Format_BIN, &rtc_DateStruct );
   RTC_WriteBackupRegister( RTC_BKP_DR0, 0x32f2 );   // #define RTC_SET_KEY	0xcd12 
	
}

void rtc_GetTimeDate( timedate* ptimdat )   //
{   //uint8_t timedate[7];  // hour,minute,second, year,month,day, weekday 
  RTC_TimeTypeDef rtc_TimeStruct;
  RTC_DateTypeDef rtc_DateStruct;
		
  RTC_GetTime( RTC_Format_BIN, &rtc_TimeStruct );
  RTC_GetDate( RTC_Format_BIN, &rtc_DateStruct );
	
  ptimdat->hour = rtc_TimeStruct.RTC_Hours;
  ptimdat->minute = rtc_TimeStruct.RTC_Minutes;
  ptimdat->second = rtc_TimeStruct.RTC_Seconds;
	
  ptimdat->year = rtc_DateStruct.RTC_Year;
  ptimdat->month = rtc_DateStruct.RTC_Month;
  ptimdat->day = rtc_DateStruct.RTC_Date;
  ptimdat->weekday = rtc_DateStruct.RTC_WeekDay;
}
//////////////////////////////////////////////////////////////////67

void rtc_GettoUnixtime( uint32_t* punixtime )
{
  struct tm timedat;
	
  RTC_TimeTypeDef rtc_TimeStruct;
  RTC_DateTypeDef rtc_DateStruct;
		
  RTC_GetTime( RTC_Format_BIN, &rtc_TimeStruct );
  RTC_GetDate( RTC_Format_BIN, &rtc_DateStruct );
	
  timedat.tm_hour = rtc_TimeStruct.RTC_Hours;
  timedat.tm_min = rtc_TimeStruct.RTC_Minutes;
  timedat.tm_sec = rtc_TimeStruct.RTC_Seconds;
	
  timedat.tm_year = rtc_DateStruct.RTC_Year +100;
  timedat.tm_mon = rtc_DateStruct.RTC_Month -1;
  timedat.tm_mday = rtc_DateStruct.RTC_Date;
	
  *punixtime = mktime( &timedat ) - 28800;  // east-8 time 28800
	
}

void rtc_SetfromUnixtime( uint32_t unixtime )
{
  struct tm* ptimedat;
  RTC_TimeTypeDef rtc_TimeStruct;
  RTC_DateTypeDef rtc_DateStruct;
	
  unixtime += 28800;   // east-8 time
  ptimedat = localtime( &unixtime );
		
  rtc_TimeStruct.RTC_H12 = RTC_HourFormat_24;
  rtc_TimeStruct.RTC_Hours = ptimedat->tm_hour;
  rtc_TimeStruct.RTC_Minutes = ptimedat->tm_min;
  rtc_TimeStruct.RTC_Seconds = ptimedat->tm_sec;
   RTC_SetTime( RTC_Format_BIN, &rtc_TimeStruct ); 
   RTC_WriteBackupRegister( RTC_BKP_DR0, 0x32f2 );   // RTC_SET_KEY	0xcd12
	
  rtc_DateStruct.RTC_Date = ptimedat->tm_mday;
  rtc_DateStruct.RTC_Month = ptimedat->tm_mon +1;
  rtc_DateStruct.RTC_Year = ptimedat->tm_year -100;   // 1900-2000
	
  #define SECONDSPERDAY  86400
  rtc_DateStruct.RTC_WeekDay = (((unixtime/SECONDSPERDAY) +4) %7);
	
  //rtc_DateStruct.RTC_WeekDay = 5;  //timedate[6];   // how to set this?
	
  RTC_SetDate( RTC_Format_BIN, &rtc_DateStruct );
  RTC_WriteBackupRegister( RTC_BKP_DR0, 0x32f2 );
	
}
//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67

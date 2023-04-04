/*
 *  liy_util_mnlog.c  
 *     mnLog functions to store run states based on flash AT45DB081E.
 *     
 *    log structure<8bytes>: unixtime(4bytes) + logCode(2bytes) + logData(2bytes) 
 *
 *  liyucai  2023-4-3
 *
 */


//////////////////////////////////////////////////////////////////67

#include <stdint.h>
#include <string.h>   
#include <stdbool.h>

//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"

#include "..\liy_mnLog\liy_drv_at45.h"
#include "..\liy_mnLog\liy_drv_spifm.h"
#include "..\liy_mnLog\liy_drv_rtc.h"
#include "..\liy_mnLog\liy_util_mnLog.h"

//////////////////////////////////////////////////////////////////67


void mnLog_Init( void )
{
  //drv_SPI2_Init();		   			
  //drv_SPI2_SetSpeed( SPI_BaudRatePrescaler_16 );		// 2.6125MHz
	
  rtc_Config( );
  at45_Config( );
	
  fm25_csInit( );
	
  mnLog_Format();
}
	
void mnLog_Format( void )   
{
  uint8_t log0[4]={0x00,0x00, 0x00,0x00};  // magic_log in page0,address0 of flash chip
  mnlog_t magic_log;
	
  uint8_t magicode[4];
  fm25_ReadBytesSussive( MAGIC_ADDRESS_FM25, magicode, 4 );
	
  if( magicode[0] != 0x64 || magicode[1] != 0x32 || magicode[2] != 0xab || magicode[3] != 0xcd )
  {
	  // first do something????
    magicode[0] =0x64; magicode[1] =0x32; magicode[2] =0xab; magicode[3] =0xcd;
  }
	
  uint8_t logx[8];
  at45_ReadBytes( 0x00, 0x00, logx, 8 );
	
  if( logx[0] != 0xcd || logx[1] != 0xab || logx[2] != 0x32 || logx[3] != 0x64 
	   || logx[4] != 0x3b || logx[5] != 0xf9 || logx[6] != 0xb3 || logx[7] != 0x9f )
  {
    at45_ErasePage( 0x0000 );   // page 0
	
    //rtc_GettoUnixtime( &(magic_log.unixtime) );
	
    magic_log.unixtime = 0x6432abcd;   // 2023-4-9,20:13:01
    magic_log.logcode = 0xf93b;
    magic_log.logdata = 0x9fb3;
	  at45_Write8Bytes( 0x00, 0x00, (uint8_t*)&magic_log );
	
		fm25_WriteBytesSussive( MAGIC_ADDRESS_FM25, magicode, 4 );
	  fm25_WriteBytesSussive( LOG_ADDRESS_LASTWRITE, log0, 4 );
	  fm25_WriteBytesSussive( LOG_ADDRESS_LASTREAD, log0, 4 );
	}
	
}

void mnLog_WriteLog( uint16_t logcode, uint16_t logdata )
{
  uint8_t logaddr[4];
	uint16_t nowpage;
	uint16_t nowbyteaddr;
	
	mnlog_t log;
	rtc_GettoUnixtime( &(log.unixtime) );
	log.logcode = logcode;
	log.logdata = logdata;
		
	fm25_ReadBytesSussive( LOG_ADDRESS_LASTWRITE, logaddr, 4 );
	nowpage = *(uint16_t*)(&logaddr[0]);
	nowbyteaddr = *(uint16_t*)(&logaddr[2]);
	nowbyteaddr += 8;   // one log use 8-byte address space
	if( nowbyteaddr > 249 )   // current page has exhausted
	{ 
		nowpage++; nowbyteaddr = 0;
	  if( nowpage > (AT45DB08_NPAGES-1) )
		{ nowpage = 0; nowbyteaddr = 8; }   // magic log in page0,address0
		at45_ErasePage( nowpage );
	}
		
	at45_Write8Bytes( nowpage, nowbyteaddr, (uint8_t*)&log );
	
	*(uint16_t*)(&logaddr[0]) = nowpage;
	*(uint16_t*)(&logaddr[2]) = nowbyteaddr;
	fm25_WriteBytesSussive( LOG_ADDRESS_LASTWRITE, logaddr, 4 );
	
}

// return true: there are new log should be read
//        false:  there are no new log 
uint8_t mnLog_CheckNewLog( void )
{
  uint8_t logaddr[4];
	uint8_t logaddr2[4];
	uint16_t nowpage;
	uint16_t nowbyteaddr;
	uint16_t nowpage2;
	uint16_t nowbyteaddr2;
		
	fm25_ReadBytesSussive( LOG_ADDRESS_LASTWRITE, logaddr, 4 );
	fm25_ReadBytesSussive( LOG_ADDRESS_LASTREAD, logaddr2, 4 );
	nowpage = *(uint16_t*)(&logaddr[0]);
	nowbyteaddr = *(uint16_t*)(&logaddr[2]);
	nowpage2 = *(uint16_t*)(&logaddr2[0]);
	nowbyteaddr2 = *(uint16_t*)(&logaddr2[2]);
	
	if( nowbyteaddr2 != nowbyteaddr || nowpage2 != nowpage )
	{
	  return true;
	}
	else
		return false;
}

void mnLog_ReadNewLog( mnlog_t* plog )
{
  uint8_t logaddr[4];
	uint8_t logaddr2[4];
	uint16_t nowpage;
	uint16_t nowbyteaddr;
	uint16_t nowpage2;
	uint16_t nowbyteaddr2;
	//mnlog_t log;
	
	fm25_ReadBytesSussive( LOG_ADDRESS_LASTWRITE, logaddr, 4 );
	fm25_ReadBytesSussive( LOG_ADDRESS_LASTREAD, logaddr2, 4 );
	nowpage = *(uint16_t*)(&logaddr[0]);
	nowbyteaddr = *(uint16_t*)(&logaddr[2]);
	nowpage2 = *(uint16_t*)(&logaddr2[0]);
	nowbyteaddr2 = *(uint16_t*)(&logaddr2[2]);
	
	if( nowbyteaddr2 != nowbyteaddr || nowpage2 != nowpage )
	{
	  
		nowbyteaddr2 += 8;   // one log use 8-byte address space
	  if( nowbyteaddr2 > 249 )   // current page has exhausted
	  { 
		  nowpage2++;  nowbyteaddr2 = 0; 
	    if( nowpage2 > (AT45DB08_NPAGES-1) )
		  { nowpage2 = 0; nowbyteaddr2 = 8;  }
		  
		}	
		at45_ReadBytes( nowpage2, nowbyteaddr2, (uint8_t*)plog, 8 );
		
		*(uint16_t*)(&logaddr[0]) = nowpage2;
	  *(uint16_t*)(&logaddr[2]) = nowbyteaddr2;
	  fm25_WriteBytesSussive( LOG_ADDRESS_LASTREAD, logaddr, 4 );
	  
	}
}
//////////////////////////////////////////////////////////////////67

/////////////////////////////////////////////////////////////////////////////78


//////////////////////////////////////////////////////////////////67




//////////////////////////////////////////////////////////////////67

//////////////////////////////////////////////////////////////////67




//////////////////////////////////////////////////////////////////67

/////////////////////////////////////////////////////////////////////////////78



//////////////////////////////////////////////////////////////////67

//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67

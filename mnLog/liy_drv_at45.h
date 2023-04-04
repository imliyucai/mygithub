/*
 *  liy_drv_at45.h   
 *  header file of driver to AT45DB081E spi flash chip
 *  
 *  liyucai  2023-3-25
 *
 */

#ifndef __LIY_DRV_AT45_H	
#define __LIY_DRV_AT45_H

#ifdef __cplusplus
 extern "C" {
#endif

//////////////////////////////////////////////////////////////////67
//#include <stdint.h>
//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"


#define  AT45DB08_NPAGES   4096

#define CMD_CHIPID        0x9f    

#define PAGE_SIZE_256BYTES  0xa6
#define PAGE_SIZE_264BYTES  0xa7

#define CMD_WRITE_BUFFER1   0x84					
#define CMD_WRITE_BUFFER2   0x87					
#define CMD_READ_BUFFER1_HF   0xd4					
#define CMD_READ_BUFFER2_HF   0xd6					

#define CMD_BYTEPROG_FLASH_THROUGHBUFFER1_NOERASE    0x02
#define CMD_PAGEPROG_FLASH_THROUGHBUFFER1_WITHERASE    0x82	
#define CMD_PAGEPROG_FLASH_THROUGHBUFFER2_WITHERASE    0x85	
#define CMD_PAGEPROG_BUFFER1_TO_FLASH_WITHERASE     0x83	
#define CMD_PAGEPROG_BUFFER2_TO_FLASH_WITHERASE     0x86	
#define CMD_PAGEPROG_BUFFER1_TO_FLASH_NOERASE    0x88	
#define CMD_PAGEPROG_BUFFER2_TO_FLASH_NOERASE    0x89	

#define CMD_PAGEREAD_FLASH_TO_BUFFER1   0x53				
#define CMD_PAGEREAD_FLASH_TO_BUFFER2   0x55				

#define CMD_PAGEREAD_FLASH    0xd2
#define CMD_READ_CONTINUOUS_HF   0x0b   
#define CMD_READ2_CONTINUOUS_HF   0x1b   

#define CMD_PAGEERASE     0x81	
#define CMD_BLOCKERASE     0x50
#define CMD_SECTORERASE     0x7c		

#define CMD_STATUSREG_READ   0xd7			
#define CMD_SOFTRESET       0xf0
 

void at45_csInit( void );

void at45_ReadId( uint8_t* theid );
void at45_SetPagesize( uint8_t pagesize );
void at45_softReset( void );

void at45_Config( void ); 
void at45_ReadStatus( uint8_t* thestatus );

void at45_WriteBuffer1( uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes );
void at45_ProgramPagefromBuffer1( uint16_t mpage );

void at45_WriteBuffer( uint8_t wrbufx, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes );
void at45_ProgramPagefromBuffer( uint8_t prgfrmbufx, uint16_t mpage );

void at45_WriteBytes( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes );
void at45_Write8Bytes( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes );

void at45_ReadBytes( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes );
void at45_ReadBytes2( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes );

void at45_ErasePage( uint16_t mpage );
void at45_EraseBlock( uint16_t mblock );
void at45_EraseAll( void );

//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67
#ifdef __cplusplus
}
#endif

#endif   /* __LIY_DRV_AT45_H */




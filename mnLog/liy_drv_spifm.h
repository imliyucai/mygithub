/*
 *  liy_drv_spifm.h   
 *  header file of driver to FM25L16 spi FRAM chip
 *  
 *  liyucai  2023-3-27
 *
 */

#ifndef __LIY_DRV_SPIFM_H	
#define __LIY_DRV_SPIFM_H

#ifdef __cplusplus
 extern "C" {
#endif

//////////////////////////////////////////////////////////////////67
//#include <stdint.h>
//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"


// FM25L16 Command
#define  FRAM_WREN    0x06
#define  FRAM_WRDI    0x04
#define  FRAM_WRSR    0x01
#define  FRAM_RDSR    0x05  
#define  FRAM_WRITE   0x02 
#define  FRAM_READ    0x03

//////////////////////////////////////////////////////////////////67
uint8_t SPI2_ReadByte( void );
uint8_t SPI2_WriteByte( uint8_t txdata );

void fm25_csInit( void );

void fm25_EnableWrite( void );
void fm25_DisableWrite( void );

uint8_t fm25_ReadStatusReg( void );
void fm25_WriteStatusReg( uint8_t stas );

void fm25_WriteByte( uint16_t addr, uint8_t tdata );
uint8_t fm25_ReadByte( uint16_t addr );    

void fm25_WriteBytesSussive( uint16_t addr, uint8_t* pdata, uint16_t nbytes );
void fm25_ReadBytesSussive( uint16_t addr, uint8_t* pdata, uint16_t nbytes );



//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67
#ifdef __cplusplus
}
#endif

#endif   /* __LIY_DRV_SPIFM_H */




/*
 *  liy_drv_spifm.c  
 *     FM25L16 is spi interface FRAM chip with 2K bytes capacity.
 *    byte address: 0~0x7ff 
 *    
 *
 *  liyucai  2023-3-30
 *
 */


//////////////////////////////////////////////////////////////////67

#include <stdint.h>
#include <string.h>   

//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"


#include  "..\liy_mnLog\liy_drv_spifm.h"

//////////////////////////////////////////////////////////////////67
uint8_t SPI2_ReadWriteByte( uint8_t txdata )
{		 			 
  while( SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_TXE ) == RESET )
  { }   // wait last tx over  
	
	SPI_I2S_SendData( SPI2, txdata );    // send one byte
	
  while( SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_RXNE ) == RESET )
  {  }  // wait data received  
 
	return SPI_I2S_ReceiveData( SPI2 );   // get in the received byte	
}

uint8_t SPI2_WriteByte( uint8_t txdata )
{		 			 
  while( SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_TXE ) == RESET )
  { }   // wait last tx over  
	
	SPI_I2S_SendData( SPI2, txdata );    // send one byte
	
  while( SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_RXNE ) == RESET )
  {  }  // wait data received  
 
	return SPI_I2S_ReceiveData( SPI2 );   // get in the received byte	
}
uint8_t SPI2_ReadByte( void )
{
  return SPI2_WriteByte( 0xff );
}

//////////////////////////////////////////////////////////////////67

void fm25_csInit( void )
{
  GPIO_InitTypeDef  gpio_InitStruct;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );

  gpio_InitStruct.GPIO_Pin = GPIO_Pin_9;
  gpio_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  gpio_InitStruct.GPIO_OType = GPIO_OType_PP;
  gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init( GPIOD, &gpio_InitStruct );
    
  GPIO_SetBits( GPIOD, GPIO_Pin_9 );
	
}

static inline void fm25_Select( void ) 
{
  GPIO_ResetBits( GPIOD, GPIO_Pin_9 );
}
static inline void fm25_deSelect( void ) 
{
  GPIO_SetBits( GPIOD, GPIO_Pin_9 );
}
//////////////////////////////////////////////////////////////////67

void fm25_EnableWrite( void )
{
 	fm25_Select();
	
  SPI2_WriteByte( FRAM_WREN );
    
	fm25_deSelect();
}

void fm25_DisableWrite( void )
{
  fm25_Select();
	
  SPI2_WriteByte( FRAM_WRDI );
    
	fm25_deSelect();
}


uint8_t fm25_ReadStatusReg( void )          
{
  uint8_t stas;
    
	fm25_Select();
	
	SPI2_WriteByte( FRAM_RDSR );
	stas = SPI2_ReadByte( );
	
	fm25_deSelect();
	
  return stas;
}                                   

void fm25_WriteStatusReg( uint8_t stas )
{
 	fm25_EnableWrite();
	
	fm25_Select();
	
	SPI2_WriteByte( FRAM_WRSR );
	SPI2_WriteByte( stas );
	
	fm25_deSelect();
}

void fm25_WriteByte( uint16_t addr, uint8_t tdata )   
{
 	fm25_EnableWrite();
	
	fm25_Select();
	
	SPI2_WriteByte( FRAM_WRITE );
	SPI2_WriteByte( (addr&0xff00)>>8 );
	SPI2_WriteByte( addr&0x00ff );
	SPI2_WriteByte( tdata );
	
	fm25_deSelect();
}

uint8_t fm25_ReadByte( uint16_t addr )     
{
  uint8_t tdata;
    
	fm25_Select();
	
	SPI2_WriteByte( FRAM_READ );
	SPI2_WriteByte( (addr&0xff00)>>8 );
	SPI2_WriteByte( addr&0x00ff );
	tdata = SPI2_ReadByte( );
	
	fm25_deSelect();
  return tdata;
}

void fm25_WriteBytesSussive( uint16_t addr, uint8_t* pdata, uint16_t nbytes )
{
  int i;
	
	fm25_EnableWrite();
	
	fm25_Select();
	for( i=0; i<13; i++ ) { ; }
	
	SPI2_WriteByte( FRAM_WRITE );
	SPI2_WriteByte( (addr&0xff00)>>8 );
	SPI2_WriteByte( addr&0x00ff );
	
	for( i=0; i<nbytes; i++ )
	{
	  SPI2_WriteByte( pdata[i] );
		addr++;
  }
		
	fm25_deSelect();

}

void fm25_ReadBytesSussive( uint16_t addr, uint8_t* pdata, uint16_t nbytes )
{
  int i;
	
	fm25_Select();
	for( i=0; i<13; i++ ) { ; }
	
	SPI2_WriteByte( FRAM_READ );
	SPI2_WriteByte( (addr&0xff00)>>8 );
	SPI2_WriteByte( addr&0x00ff );
		
	for( i=0; i<nbytes; i++ )
	{	
	  pdata[i] = SPI2_ReadByte( );
		addr++;
  }
		
	fm25_deSelect();  
}

//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67

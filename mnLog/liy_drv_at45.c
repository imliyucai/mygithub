/*
 *  liy_drv_at45.c  
 *     AT45DB081E is spi interface flash with 1M bytes capacity.
 *    sector=32block=256page      block=8page      page=256Byte|264Byte 
 *    all: 16 sectors = 512 blocks = 4096 pages = 1M bytes
 *
 *  liyucai  2023-3-25
 *
 */


//////////////////////////////////////////////////////////////////67

#include <stdint.h>
#include <string.h>   

//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"

#include "..\liy_mnLog\liy_drv_at45.h"
#include "..\liy_mnLog\liy_drv_spifm.h"

//////////////////////////////////////////////////////////////////67

void at45_csInit( void )
{
  GPIO_InitTypeDef  gpio_InitStruct;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );

  gpio_InitStruct.GPIO_Pin = GPIO_Pin_8;
  gpio_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  gpio_InitStruct.GPIO_OType = GPIO_OType_PP;
  gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init( GPIOD, &gpio_InitStruct );
  
  GPIO_SetBits( GPIOD, GPIO_Pin_8 );
}

static inline void at45_Select( void ) 
{
  GPIO_ResetBits( GPIOD, GPIO_Pin_8 );
}
static inline void at45_deSelect( void ) 
{
  GPIO_SetBits( GPIOD, GPIO_Pin_8 );
}

//////////////////////////////////////////////////////////////////67

/////////////////////////////////////////////////////////////////////////////78
void at45_ReadId( uint8_t* theid )
{
  at45_Select();
	
  SPI2_WriteByte( CMD_CHIPID );
  theid[0] = SPI2_ReadByte( );
  theid[1] = SPI2_ReadByte( );
  theid[2] = SPI2_ReadByte( );
  //theid[3] = SPI2_ReadWriteByte( 0xff );
  //theid[4] = SPI2_ReadWriteByte( 0xff );
	
  at45_deSelect();
} 

void at45_softReset( void )
{
  at45_Select();
	
  SPI2_WriteByte( CMD_SOFTRESET );
  SPI2_WriteByte( 0x00 );
  SPI2_WriteByte( 0x00 );
  SPI2_WriteByte( 0x00 );
	
  at45_deSelect();
}

void at45_SetPagesize( uint8_t pagesize )
{                      // PAGE_SIZE_256BYTES / PAGE_SIZE_264BYTES 
  if( pagesize != PAGE_SIZE_256BYTES 
        && pagesize != PAGE_SIZE_264BYTES )
  { return; }	
	
  at45_Select();
	
  SPI2_WriteByte( 0x3d );
  SPI2_WriteByte( 0x2a );
  SPI2_WriteByte( 0x80 );
	
  SPI2_WriteByte( pagesize );   // 0xa6(256bytes) 0xa7(264bytes)
	
  at45_deSelect();
}

//////////////////////////////////////////////////////////////////67
void at45_Config( void ) 
{
  at45_csInit();
  at45_softReset();
	
  at45_SetPagesize( PAGE_SIZE_256BYTES );
}

/*
 *  status register1  [rdy/busy#][compareResult][1001=8Mbit][sectorProtect][264byte/256byte#]
 *  status register2  [rdy/busy#][0][erase_programError][0][sectorLockdown][programSuspend_buffer2][programSuspend_buffer1][eraseSuspend]
 */
void at45_ReadStatus( uint8_t* thestatus )
{
  at45_Select();
	
  SPI2_WriteByte( CMD_STATUSREG_READ );
  thestatus[0] = SPI2_ReadByte( );
  thestatus[1] = SPI2_ReadByte( );
	
  at45_deSelect();
}
//////////////////////////////////////////////////////////////////67

void at45_WriteBuffer( uint8_t wrbufx, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes )
{                     // CMD_WRITE_BUFFER1/2
  int i;
  uint8_t tstas;
	
  if( wrbufx != CMD_WRITE_BUFFER1 
        && wrbufx != CMD_WRITE_BUFFER2 )
  { return; }	
	
  at45_ReadStatus( &tstas );
  if( (tstas&0x80) == 0x80 )
  {
    at45_Select();
	
    SPI2_WriteByte( wrbufx );  // CMD_WRITE_BUFFER1/2
    SPI2_WriteByte( 0x00 );
    SPI2_WriteByte( 0x00 );
    SPI2_WriteByte( addressinpage );   // 
	
    for( i=0; i<nbytes; i++ )
    { SPI2_WriteByte( pbytes[i] ); }
	
    at45_deSelect();
  }
}
void at45_ProgramPagefromBuffer( uint8_t prgfrmbufx, uint16_t mpage )
{                         // CMD_PAGEPROG_BUFFER1_TO_FLASH_NOERASE
  int i;
  uint8_t tstas;
	
  if( prgfrmbufx != CMD_PAGEPROG_BUFFER1_TO_FLASH_NOERASE 
        && prgfrmbufx != CMD_PAGEPROG_BUFFER2_TO_FLASH_NOERASE )
  { return; }	
	
  at45_ReadStatus( &tstas );
  if( (tstas&0x80) == 0x80 )
  {
    at45_Select();  // 
	
    SPI2_WriteByte( prgfrmbufx );  // CMD_PAGEPROG_BUFFER1_TO_FLASH_NOERASE
    SPI2_WriteByte( (mpage & 0xff00)>>8 );
    SPI2_WriteByte( mpage & 0x00ff );
    SPI2_WriteByte( 0x00 );   // 
	
    at45_deSelect();
    for( i=0; i<1234; i++ ) { ; }
  }
}
//////////////////////////////////////////////////////////////////67
// this command maybe any wrong, which write last byte only ??
void at45_WriteBytes( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes )
{
  int i,j;  
  uint8_t tstas;
	
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 ) { at45_ReadStatus( &tstas ); }
	
  //if( (tstas&0x80) == 0x80 )
  {
    at45_Select();
	
    SPI2_WriteByte( CMD_BYTEPROG_FLASH_THROUGHBUFFER1_NOERASE );
    SPI2_WriteByte( (mpage & 0xff00)>>8 );
    SPI2_WriteByte( mpage & 0x00ff );
    SPI2_WriteByte( addressinpage );
	
    // data bytes in
    for( i=0; i<nbytes; i++ )
    { 
      SPI2_WriteByte( pbytes[i] ); 
      //for(j=0; j<178; j++){;}
    }
	
    at45_deSelect();
  }
  for( i=0; i<173; i++)	{ ; }
	
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 )  { at45_ReadStatus( &tstas ); }   
}

void at45_Write8Bytes( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes )
{
  int i,j;  
  uint8_t tstas;
	
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 ) { at45_ReadStatus( &tstas ); j++; if(j>0x8000) return; }
	
  for( i=0; i<8; i++ )
  {
    at45_Select();
	
    SPI2_WriteByte( CMD_BYTEPROG_FLASH_THROUGHBUFFER1_NOERASE );
    SPI2_WriteByte( (mpage & 0xff00)>>8 );
    SPI2_WriteByte( mpage & 0x00ff );
    SPI2_WriteByte( addressinpage+i );
	
    // data bytes in
    SPI2_WriteByte( pbytes[i] ); 
	  	  
    at45_deSelect();
		
    for(j=0; j<13; j++) { ; }
    at45_ReadStatus( &tstas );
    while( (tstas&0x80) != 0x80  ) { at45_ReadStatus( &tstas ); j++; if(j>0x8000) return; }   
  }
  for( i=0; i<31; i++)	{ ; }
}
//////////////////////////////////////////////////////////////////67

void at45_ReadBytes( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes )
{
  int j=0;
  uint8_t tstas;
		
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 ) { at45_ReadStatus( &tstas ); j++; if(j>0x8000) return; }
	
  //if( (tstas&0x80) == 0x80 )
  {
    at45_Select();
		
    SPI2_WriteByte( CMD_READ_CONTINUOUS_HF );
    SPI2_WriteByte( (mpage & 0xff00)>>8 );
    SPI2_WriteByte( mpage & 0x00ff );
    SPI2_WriteByte( addressinpage );
    SPI2_WriteByte( 0x00 );
	
    // data bytes out
    for( int i=0; i<nbytes; i++ )
    { pbytes[i] = SPI2_ReadByte( ); }
  
    at45_deSelect();
  }
}

// this command maybe any wrong, ??
void at45_ReadBytes2( uint16_t mpage, uint16_t addressinpage, uint8_t* pbytes, uint16_t nbytes )
{
  int j=0;
  uint8_t tstas;
		
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 ) { at45_ReadStatus( &tstas ); j++; if(j>0x8000) return; }
	
  //if( (tstas&0x80) == 0x80 )
  {
    at45_Select();
	
    SPI2_WriteByte( CMD_READ2_CONTINUOUS_HF );
    SPI2_WriteByte( (mpage & 0xff00)>>8 );
    SPI2_WriteByte( mpage & 0x00ff );
    SPI2_WriteByte( addressinpage );
    SPI2_WriteByte( 0x00 );  SPI2_WriteByte( 0x00 );
	
    // data bytes out
    for( int i=0; i<nbytes; i++ )
    { pbytes[i] = SPI2_ReadByte( ); }
	
    at45_deSelect();
  }
}

//////////////////////////////////////////////////////////////////67
void at45_ErasePage( uint16_t mpage )
{                   // mpage = [0,4096)
  int j=0;
  uint8_t tstas;
	
  at45_Select();
	
  SPI2_WriteByte( CMD_PAGEERASE );
  SPI2_WriteByte( (mpage & 0xff00)>>8 );
  SPI2_WriteByte( mpage & 0x00ff );
  SPI2_WriteByte( 0x00 );
	
  at45_deSelect();
  for( j=0; j<985; j++ )  { ; }
	
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 ) { at45_ReadStatus( &tstas ); }
}

void at45_EraseBlock( uint16_t mblock )
{                   // mblock = [0,512)
  int j=0;
  uint8_t tstas;
	
  at45_Select();
	
  SPI2_WriteByte( CMD_BLOCKERASE );
  SPI2_WriteByte( (mblock>>5) & 0x00ff );
  SPI2_WriteByte( (mblock<<3) & 0x00ff );  //0x00f8
  SPI2_WriteByte( 0x00 );
	
  at45_deSelect();
  for( j=0; j<1024; j++ )  { ; }
	
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 ) { at45_ReadStatus( &tstas ); }
}

void at45_EraseAll( void )
{                   
  int j=0;
  uint8_t tstas;
	
  at45_Select();
	
  SPI2_WriteByte( 0xc7 );
  SPI2_WriteByte( 0x94 );
  SPI2_WriteByte( 0x80 );  
  SPI2_WriteByte( 0x9a );
	
  at45_deSelect();
	
  for( j=0; j<1234; j++ )  { ; }
	
  at45_ReadStatus( &tstas );
  while( (tstas&0x80) != 0x80 ) { at45_ReadStatus( &tstas ); }
}

/////////////////////////////////////////////////////////////////////////////78



//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67

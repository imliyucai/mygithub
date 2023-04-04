/*
 *  liy_task_mnLog.c  
 *     this task for run log writing and reading 
 *    
 *    
 *
 *  liyucai  2023-3-27
 *
 */


//////////////////////////////////////////////////////////////////67

#include <stdint.h>
#include <string.h>   
// for bool: true false
#include <stdbool.h>


//uC-OS header file
#include "includes.h"
#include "os_cfg_app.h"
#include "app_task_config.h"

//////////////////////////////////////////////////////////////////67


#include  "..\liy_mnLog\liy_task_mnLog.h"
#include  "..\liy_mnLog\liy_drv_at45.h"
#include  "..\liy_mnLog\liy_drv_spifm.h"
#include  "..\liy_mnLOg\liy_drv_rtc.h"
#include  "..\liy_mnLog\liy_util_mnLog.h"

#include "stm32f4xx_gpio.h"





OS_TCB  liy_mnLog_TaskTCB;	
CPU_STK  liy_mnLog_TaskSTK[ 1024 ];  

//OS_Q    MotorCAN_LeftRecvQ;
//OS_Q    MotorCAN_RightRecvQ;
//OS_Q    MotorCAN_SentQ;	

//extern uint16_t  liy_uart_RcvCnt;
//extern uint8_t  liy_uart_RcvbkBuf[];

//extern uint8_t  liy_uart1_txbuf[]; 

//#define  UART1_DMA_TX_ENABLE
#define  UART1_DMA_RX_ENABLE
//#define  UART1_DMA_RX_INT_ENABLE

uint8_t  liy_uart1_txbuf[ UART1_TX_DMA_BUFSIZE ]; 
uint8_t  liy_uart1_rxbuf[ UART1_RX_DMA_BUFSIZE ]; 

uint8_t  liy_uart_RcvbkBuf[UART1_RX_DMA_BUFSIZE+16];
uint16_t  liy_uart_RcvCnt;
///////////////////////////////////



/////////////////////////////////////////////////////////////////////////////78

uint32_t nowtime;
mnlog_t  telog;

//////////////////////////////////////////////////////////////////67
void liy_task_mnLog( void* p_arg )
{
	p_arg = p_arg;

	OS_ERR err;
  OS_MSG_SIZE size;
	
	//uint8_t cantx_buf[] = { 0x23, 0x25, 0x27, 0x29, 0x2b, 0x2d, 0x2f, 0x2e };
	
	timedate timdat;
  uint8_t tied[32];
	uint8_t thed[256];  
	int i;
	for( i=0; i<256; i++ ){ thed[i]=i+3; }
	thed[255]=0x78;
	
	UART1tohost_Init( UARTBAUDRATE_115200 );
	
///////////////////////////////////////////////////////56	
	mnLog_Init();
	
	//fm25_csInit();
	//at45_Config( );  //at45_csInit();
	//rtc_Config( );	
	
	
	timdat.hour=16; timdat.minute=10; timdat.second=0; 
	timdat.year=23; timdat.month=4; timdat.day=4; timdat.weekday=2;
	rtc_SetTimeDate( &timdat );
	//rtc_SetfromUnixtime( 0x64300000 );
	
	//at45_ReadId( theid );
	UART1tohost_SendData( (uint8_t*)&thed, 2 );
		
	
	uint16_t mpage=0;  
	
	//at45_ErasePage( 3 );
	//at45_EraseAll( );
	
	//at45_WriteBuffer( CMD_WRITE_BUFFER1, 0x00, thed, 256 );
	//at45_ProgramPagefromBuffer( CMD_PAGEPROG_BUFFER1_TO_FLASH_NOERASE, 4095 );
		
	//at45_Write8Bytes( 13, 0x40, cantx_buf );
	
	//for( i=0; i<8; i++ )
	//{ at45_WriteBytes( 13, 0x10+i, &(cantx_buf[i]), 1 ); }
	
	//at45_WriteBytes( 23, 0x40, cantx_buf, 8 );
	
	//fm25_WriteByte( 0x138, cantx_buf[3] );
  //uint8_t fm25_ReadByte( uint16_t addr ); 
  //fm25_WriteBytesSussive( 573, thed, 128 );
	
	
	while(1)
	{
		OSTimeDly( OS_CFG_TICK_RATE_HZ*1, OS_OPT_TIME_DLY, &err );
		UART1tohost_SendData( (uint8_t*)(&thed[2]), 3 );
    
		OSTimeDly( OS_CFG_TICK_RATE_HZ*13, OS_OPT_TIME_DLY, &err );
		mnLog_WriteLog( 0x0327, 0xd1d2 );
		UART1tohost_SendData( (uint8_t*)(&thed[5]), 4 );
		
		
		OSTimeDly( OS_CFG_TICK_RATE_HZ*2, OS_OPT_TIME_DLY, &err );
		rtc_GetTimeDate( &timdat );
		UART1tohost_SendData( (uint8_t*)&timdat, sizeof(timdat) );
		
		OSTimeDly( OS_CFG_TICK_RATE_HZ*1, OS_OPT_TIME_DLY, &err );
		rtc_GettoUnixtime( &nowtime );
		UART1tohost_SendData( (uint8_t*)&nowtime, 4 );
	

		OSTimeDly( OS_CFG_TICK_RATE_HZ*2, OS_OPT_TIME_DLY, &err );
    while( mnLog_CheckNewLog() == true )
    {
		  mnLog_ReadNewLog( &telog );
			UART1tohost_SendData( (uint8_t*)(&telog), 8 );
		}
		
					
//////////////////////////////////////////////////////////////////67		
		OSTimeDly( OS_CFG_TICK_RATE_HZ/1, OS_OPT_TIME_DLY, &err );
		fm25_ReadBytesSussive( MAGIC_ADDRESS_FM25, tied, 4*3 );
		UART1tohost_SendData( tied, 4*3 ); 
		
		OSTimeDly( OS_CFG_TICK_RATE_HZ/1, OS_OPT_TIME_DLY, &err );
		at45_ReadBytes( 0, 0x00, thed, 256 );
		UART1tohost_SendData( thed, 8 ); 
		
		OSTimeDly( OS_CFG_TICK_RATE_HZ/1, OS_OPT_TIME_DLY, &err );
		at45_ReadBytes( 4, 0x00, thed, 256 );
		for( i=0; i<256; i++ )
		{ 
			if( thed[i] != 0xff )
		  { 
				UART1tohost_SendData( &thed[i], 1 ); 
		//		UART1tohost_SendData( (uint8_t*)(&i), 1 ); 
		//	  UART1tohost_SendData( (uint8_t*)(&mpage), 2 );
			}
		}
		
		// OS_CFG_TICK_RATE_HZ=200, tick=5ms, 200*5ms=1000ms
		
		mpage++;
		//if( mpage%128 == 0 )
		//{ UART1tohost_SendData( (uint8_t*)(&mpage), 2 ); }
	}
}



//////////////////////////////////////////////////////////////////67





///////////////////////////////////////////////////////////////////////

void UART1tohost_Init( uint32_t bdrate )
{
  GPIO_InitTypeDef  gpio_InitStruct;
	USART_InitTypeDef  usart_InitStruct;
	NVIC_InitTypeDef  nvic_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE ); 	// GPIOA A9/A10 used as USART1
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );	// USART1

	USART_DeInit( USART1 );  // reset all settings 
 
	// A9/A10 used as USART1
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource9, GPIO_AF_USART1 ); 
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource10, GPIO_AF_USART1 ); 
	
	// USART1 in A9/A10
	gpio_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 	
	gpio_InitStruct.GPIO_Mode = GPIO_Mode_AF;			       // 
	gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		   // 
	gpio_InitStruct.GPIO_OType = GPIO_OType_PP; 			   // 
	gpio_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 			     // 
	GPIO_Init( GPIOA, &gpio_InitStruct ); 					

  // USART1 parameters
	usart_InitStruct.USART_BaudRate = bdrate;										// baudrate 115200
	usart_InitStruct.USART_WordLength = USART_WordLength_8b;		// 
	usart_InitStruct.USART_StopBits = USART_StopBits_1;					// 
	usart_InitStruct.USART_Parity = USART_Parity_No;						// 
	usart_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	  // 
	usart_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// RX and TX mode
	USART_Init( USART1, &usart_InitStruct ); 				
	

#ifdef UART1_DMA_TX_ENABLE
    
		DMA_InitTypeDef  dma_InitStruct;
	
		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );    // DMA clock enable

		USART_DMACmd( USART1, USART_DMAReq_Tx, ENABLE );       // DMA for USART1 enable

		DMA_DeInit( DMA2_Stream7 );	     // No. of DMA
		
		while( DMA_GetCmdStatus(DMA2_Stream7) != DISABLE )
		{   	}
		
		dma_InitStruct.DMA_Channel = DMA_Channel_4;                 // 
		dma_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;   // 
		dma_InitStruct.DMA_Memory0BaseAddr = (uint32_t)liy_uart1_txbuf;    // 
		dma_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;	      // 
		dma_InitStruct.DMA_BufferSize = UART1_TX_DMA_BUFSIZE;         // 
		dma_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // 
		dma_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;            // 
		dma_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   // 
		dma_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           // 
		dma_InitStruct.DMA_Mode = DMA_Mode_Normal;                      // 
		dma_InitStruct.DMA_Priority = DMA_Priority_Medium;              //  
		dma_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
		dma_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		dma_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		dma_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream7, &dma_InitStruct);  
		
		nvic_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;     // DMA interrupt
		nvic_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;   // 
		nvic_InitStruct.NVIC_IRQChannelSubPriority = 3;		       // 
		nvic_InitStruct.NVIC_IRQChannelCmd = ENABLE; 		         // 
		NVIC_Init( &nvic_InitStruct );                     
		
		DMA_ClearITPendingBit( DMA2_Stream7, DMA_IT_TCIF7 );
		DMA_ITConfig( DMA2_Stream7, DMA_IT_TC, ENABLE );
	
#else
	
#endif
	
////////////////////////////////////////////////// RX setting ///////////////78 	
		
#ifdef UART1_DMA_RX_ENABLE	

 #ifdef UART1_DMA_TX_ENABLE
	
 #else
		DMA_InitTypeDef  dma_InitStruct;
 #endif
		
		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );  // DMA for USART1 RX enable

		USART_ITConfig( USART1, USART_IT_IDLE, ENABLE );   // 
		USART_DMACmd( USART1, USART_DMAReq_Rx, ENABLE );   // 

		DMA_DeInit( DMA2_Stream5 );	  
		
		while( DMA_GetCmdStatus(DMA2_Stream5) != DISABLE )
		{  		}
		
		dma_InitStruct.DMA_Channel = DMA_Channel_4;     // 
		dma_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;   // 
		dma_InitStruct.DMA_Memory0BaseAddr = (uint32_t)liy_uart1_rxbuf;    // 
	  dma_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;	       // 
		dma_InitStruct.DMA_BufferSize = UART1_RX_DMA_BUFSIZE;           // 
		dma_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // 
		dma_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;            // 
		dma_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   // 
		dma_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    // 
		dma_InitStruct.DMA_Mode = DMA_Mode_Normal;                      // 
		dma_InitStruct.DMA_Priority = DMA_Priority_Medium;              // 
		dma_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
		dma_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		dma_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		dma_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init( DMA2_Stream5, &dma_InitStruct );  
    
		DMA_Cmd( DMA2_Stream5, ENABLE );       // enable RX DMA2.5
		
		while( (DMA_GetCmdStatus(DMA2_Stream5) != ENABLE) )
		{ 		}

 #ifdef UART1_DMA_RX_INT_ENABLE

   	nvic_InitStruct.NVIC_IRQChannel = DMA2_Stream5_IRQn;       // USART1 RX DMA interrupt
		nvic_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;     
		nvic_InitStruct.NVIC_IRQChannelSubPriority = 3;		        
		nvic_InitStruct.NVIC_IRQChannelCmd = ENABLE; 		          
		NVIC_Init( &nvic_InitStruct );                 
		
		DMA_ITConfig( DMA2_Stream5, DMA_IT_TC, ENABLE );
 #endif
	
#else
		USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );    // USART1 RX interrupt( non-DMA )
		
#endif
	
	nvic_InitStruct.NVIC_IRQChannel = USART1_IRQn;           // 
	nvic_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;   // 
	nvic_InitStruct.NVIC_IRQChannelSubPriority = 3;		       
	nvic_InitStruct.NVIC_IRQChannelCmd = ENABLE; 		         // 
	NVIC_Init( &nvic_InitStruct );        

	USART_Cmd( USART1, ENABLE );             // enable USART1
	
}


///////////////////////////////////////////////////////56
void UART1tohost_SendByte( uint8_t byte )
{ 	
	while( (USART1->SR&0X40)==0 )
		;			  
	USART1->DR = (uint8_t) byte;      
}

void UART1tohost_SendData( uint8_t* sp, uint16_t length )
{
	while( length-- )
	{
		UART1tohost_SendByte( *(sp++) );
	}
	
}
void UART1tohost_SendString( char *sp )
{
    uint8_t i=0;
    while( sp[i++] != '\0' )
    {    }
    UART1tohost_SendData( (uint8_t* )sp, i-1 );
}



#ifdef UART1_DMA_TX_ENABLE
void UART1tohost_SendDataDMA( uint8_t* sp, uint16_t length )
{
    OS_ERR err;
    //OS_MSG_SIZE size;

    OSSemPend( &Comm_SentProtectSEM, 0, OS_OPT_PEND_BLOCKING, 0, &err );  
    if( length>2 )
    {
        DMA2_Stream7->NDTR = length;
        DMA2_Stream7->M0AR = (uint32_t)sp;
        DMA2_Stream7->CR |= (uint32_t)DMA_SxCR_EN; 	
        
        OSSemPend( &Comm_SentSEM, 0, OS_OPT_PEND_BLOCKING, 0, &err ); 
        if( err == OS_ERR_NONE )
        {           }
        else
        {           }
        while( USART_GetFlagStatus( USART1, USART_FLAG_TXE ) == RESET )
        {           }
        while( USART_GetFlagStatus( USART1, USART_FLAG_TC ) == RESET )
        {           }
    }
    else
    {
      while( length-- )
    	{
    		UART1tohost_SendByte( *(sp++) );
    	}
        while( USART_GetFlagStatus( USART1, USART_FLAG_TC ) == RESET )
        {
        }
    }
    OSSemPost( &Comm_SentProtectSEM, OS_OPT_POST_1, &err );

}

void UART1tohost_SendStringDMA( char* sp )
{
    uint8_t i=0;
    while( sp[i++] != '\0' )
    {    }
    UART1tohost_SendDataDMA( (uint8_t* )sp, i-1 );
}
#endif

#ifdef UART1_DMA_RX_ENABLE
void DMA2_Stream5_IRQHandler( void )
{
//    OS_ERR err;
        
  //OSIntEnter();
  if( DMA_GetITStatus( DMA2_Stream5, DMA_IT_TCIF5 ) == SET )
	{
		DMA_ClearITPendingBit( DMA2_Stream5, DMA_IT_TCIF5 );
		DMA_Cmd( DMA2_Stream5, DISABLE );  
    DMA_SetCurrDataCounter( DMA2_Stream5, UART1_RX_DMA_BUFSIZE ); // DMA buffer size
    DMA_ClearFlag( DMA2_Stream5, DMA_FLAG_TCIF5 );
    DMA_Cmd( DMA2_Stream5, ENABLE );  
        
	}
  //OSIntExit();
}


#endif

#ifdef UART1_DMA_TX_ENABLE
void DMA2_Stream7_IRQHandler( void )
{
  //OS_ERR err;
    
  //OSIntEnter();
    
	if( DMA_GetITStatus( DMA2_Stream7, DMA_IT_TCIF7 ) == SET )
	{
		DMA_ClearITPendingBit( DMA2_Stream7, DMA_IT_TCIF7 );
		
		DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN; 
    OSSemPost( &Comm_SentSEM, OS_OPT_POST_1, &err );
 
	}
  //OSIntExit();
}
#endif

void USART1_IRQHandler( void )                	
{
  //OS_ERR err;

  //OSIntEnter();

#ifdef UART1_DMA_RX_ENABLE	
	if( USART_GetITStatus( USART1, USART_IT_IDLE ) != RESET ) 
	{
		USART_ClearITPendingBit( USART1, USART_IT_IDLE ); 
		USART_ReceiveData( USART1 );     //must read DR,so that to clear the IDLE Flag
		
		// now maybe any received data in RXDMA_BUFFER
		liy_uart_RcvCnt = UART1_RX_DMA_BUFSIZE - DMA_GetCurrDataCounter( DMA2_Stream5 ); 
		
//UART1tohost_SendData( liy_uart1_rxbuf, 8 );		// for test only
		
		//CopyBuffer( liy_uart1_rxbuf, liy_uart_RcvbkBuf, liy_uart_RcvCnt );
		for( uint16_t i=0; i<liy_uart_RcvCnt; i++ )
		{ liy_uart_RcvbkBuf[i] = liy_uart1_rxbuf[i];  }  
		

		DMA_Cmd( DMA2_Stream5, DISABLE );    
		DMA_SetCurrDataCounter( DMA2_Stream5, UART1_RX_DMA_BUFSIZE ); 
		DMA_ClearFlag( DMA2_Stream5, DMA_FLAG_TCIF5 );
		DMA_Cmd( DMA2_Stream5, ENABLE );  

            #if USART1_COMM_MODE_EN
            OSQPost((OS_Q*		)&Comm_RecvQ,		
					(void*		)&USART1_Rrcv,
					(OS_MSG_SIZE)(sizeof(RECV_STRUCT)),
					(OS_OPT		)OS_OPT_POST_FIFO,
					(OS_ERR*	)&err);
            #else

            #endif
	}
#else

	if( USART_GetITStatus( USART1, USART_IT_RXNE ) != RESET )  
	{
		liy_uart1_rxbuf[Res] = USART_ReceiveData( USART1 );
		UART1tohost_SendByte( liy_uart1_rxbuf[Res++] );
  } 
#endif

  //OSIntExit();

} 


void liy_Uart1_Test( void )
{
  uint8_t xdat[8]={ 0x64, 0xff, 0xff, 0xff, 0xff, 0xff, 0x71, 0x71 };
	
	//UART1tohost_Init( 115200 );
  	
	
	//for( int i=0; i<7; i++)
	  UART1tohost_SendData( xdat, 8 );
	
	if( liy_uart_RcvCnt > 0 )
	{
		
	  UART1tohost_SendData( liy_uart_RcvbkBuf, liy_uart_RcvCnt );
		
		liy_uart_RcvCnt = 0;
		
	}
	
}

//////////////////////////////////////////////////////////////////67


//////////////////////////////////////////////////////////////////67



//////////////////////////////////////////////////////////////////67

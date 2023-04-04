/*
 *   the driver for LIS3DH and some related functions.         
 *   LIS3DH is a 3-axis accelerometer used as motion sensor.
 *
 *   liyucai  2022-12-24
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"

//#include "nrf_drv_uart.h"

#include "app_timer.h"

#include "es_app_config.h"

#include "es_adv_timing.h"
#include "es_adv_timing_resolver.h"
#include "es_tlm.h"
#include "es_slot.h"

#include "es_adv.h"

#include "LIS3DH.h"


static const nrf_drv_twi_t  twilis3d = NRF_DRV_TWI_INSTANCE( TWI_INSTANCE_ID );
const nrf_drv_twi_config_t  twilis3d_cfg = {
                .scl    = TWI0M_SCL,
                .sda    = TWI0M_SDA,
                .frequency  = NRF_TWI_FREQ_100K,
                .interrupt_priority   = TWI_DEFAULT_CONFIG_IRQ_PRIORITY,
                .clear_bus_init    = false        };


void twi0m_Init( void )
{
  ret_code_t  err_code;

  nrf_gpio_cfg_output( LIS3D_CS );
  //nrf_gpio_cfg_output( LIS3D_SA0 );   // need not ?

    // nrf_gpio_cfg_input( LIS3D_INT1, NRF_GPIO_PIN_PULLUP );  // NRF_GPIO_PIN_NOPULL  NRF_GPIO_PIN_PULLUP
    nrf_gpio_cfg_input( LIS3D_INT1, NRF_GPIO_PIN_NOPULL );

  err_code = nrf_drv_twi_init( &twilis3d, &twilis3d_cfg, NULL, NULL );  
  APP_ERROR_CHECK( err_code );
 

  nrf_drv_twi_enable( &twilis3d );
}

void twi0m_Stop( void )
{
  //nrf_drv_twi_disable( &twilis3d );

  nrf_drv_twi_uninit( &twilis3d );

  nrf_gpio_pin_clear( LIS3D_CS );   // input can low power?
}

void LIS3DH_ReadReg( uint8_t regaddr, uint8_t* pdata )
{
  ret_code_t  err_code;
  uint8_t  dbuf[2]; 

  //nrf_gpio_pin_set( LIS3D_SA0 );   // need not ?
  nrf_gpio_pin_set( LIS3D_CS );    // set 1 to select I2C communication
  dbuf[0] = regaddr;
   //dbuf[1] = ddata;
  err_code = nrf_drv_twi_tx( &twilis3d, LIS3DH_ADDR, dbuf, 1, false );
  APP_ERROR_CHECK( err_code );
  
  err_code = nrf_drv_twi_rx( &twilis3d, LIS3DH_ADDR, pdata, 1 );
  APP_ERROR_CHECK( err_code );
    
}
void LIS3DH_WriteReg( uint8_t regaddr, uint8_t ddata )
{
  ret_code_t  err_code;
  uint8_t  dbuf[2]; 

  //nrf_gpio_pin_set( LIS3D_SA0 );
  nrf_gpio_pin_set( LIS3D_CS );

  dbuf[0] = regaddr;
   dbuf[1] = ddata;
  err_code = nrf_drv_twi_tx( &twilis3d, LIS3DH_ADDR, dbuf, 2, false );
  APP_ERROR_CHECK( err_code );
}

void LIS3DH_Config( void )
{
  // config control registers
                   // [0010][0][111]
  //LIS3DH_WriteReg( CTRL_REG1, 0x27 );    // turn on LIS3DH with ODR=10Hz normal mode, that is not low-power mode  

                     // [0010][1][111]
    LIS3DH_WriteReg( CTRL_REG1, 0x2f );    // turn on LIS3DH with ODR=10Hz low-power mode, that is low-power mode
                     // [0011][1][111]
    //LIS3DH_WriteReg( CTRL_REG1, 0x3f );    // turn on LIS3DH with ODR=25Hz low-power mode, that is low-power mode

                   // [00][00][0][001]
  LIS3DH_WriteReg( CTRL_REG2, 0x01 );    // high-pass filter enable with 0.2 cut-off frequency for INT1 interrupt generate
                   // [010][00][000]
  LIS3DH_WriteReg( CTRL_REG3, 0x40 );    // ACC AOI1(IA1) interrupt signal route to INT1 pin
                   // [10][00][1][00][0]
  LIS3DH_WriteReg( CTRL_REG4, 0x88 );    // full scale =2g with BDU and HR bits enable, data LSB at low address
                   // 
  LIS3DH_WriteReg( CTRL_REG5, 0x00 );    // interrupt signal on INT1 not latch, do not need clear interrupt signal
  
 
  // config for wakeup and motionless detection
                   // 0[0001000]
  //LIS3DH_WriteReg( INT1_THS, 0x0a );     // threshold = 8LSBs * 15.625mg/LSB = 125mg    0x0a=156mg
   LIS3DH_WriteReg( INT1_THS, 0x0f );     // threshold = 8LSBs * 15.625mg/LSB = 125mg    0x0c=190mg, 0x0e=218mg, 0x0f=234mg
                   // 0[1100100]=0x64
  LIS3DH_WriteReg( INT1_DURATION, 0x64 );    // duration = 100LSBs * (1/10Hz) = 10s
                   // 10[01][01][01]
  LIS3DH_WriteReg( INT1_CFG, 0x95 );     // enable XLIE,YLIE and ZLIE interrupt generate, when 3D is within THS threshold simultaneously
  
}


//////////////////////////////////////////////////////////////////67




//////////////////////////////////////////////////////////////////67

APP_TIMER_DEF( adv_on_timer );
#define ADV_ON_LASTTIME  ( APP_TIMER_TICKS(60000*30) ) 
static bool adv_onoff = true;

void int1_Interrupt_Config( void ) 
{
  ret_code_t  err_code;

  if( !nrf_drv_gpiote_is_init() )
  {
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK( err_code );
       //VERIFY_SUCCESS(err_code);
  }

  nrf_drv_gpiote_in_config_t  int1_cfg = GPIOTE_CONFIG_IN_SENSE_HITOLO( false );
  //int1_cfg.pull = NRF_GPIO_PIN_PULLUP;
  err_code = nrf_drv_gpiote_in_init( LIS3D_INT1, &int1_cfg, int1_evt_handle );
  APP_ERROR_CHECK( err_code );

  nrf_drv_gpiote_in_event_enable( LIS3D_INT1, true );

}
// when interrupt because of viberation, should send advertising packets 
void int1_evt_handle( nrf_drv_gpiote_pin_t  pin, nrf_gpiote_polarity_t  action )
{
  //static bool nows=true;
  ret_code_t  err_code;

                                       // 2022-12-30
  if( (pin == LIS3D_INT1) && (nrf_gpio_pin_read(LIS3D_INT1) == false ) )
  {
   // led_Toggle( LED );   // for test only  liyc 2023-2-28

    if( adv_onoff == false )
    {
                                  //es_tlm_adv_cnt_reset();   // 2022-12-26
      
      es_adv_timing_stop();
      es_adv_timing_start( APP_CFG_NON_CONN_ADV_INTERVAL_MS );
      
      //adv_restart();


      adv_onoff = true;

      err_code = app_timer_start( adv_on_timer, ADV_ON_LASTTIME, NULL );
      APP_ERROR_CHECK(err_code);
    }
    else if( adv_onoff == true )
    {
      err_code = app_timer_stop( adv_on_timer );
      APP_ERROR_CHECK(err_code);
      err_code = app_timer_start( adv_on_timer, ADV_ON_LASTTIME, NULL );
      APP_ERROR_CHECK(err_code);
    }
  }


/*  if( (nrf_gpio_pin_read(LIS3D_INT1) == false ) )
  {
    
    {
                                 
      es_adv_timing_start( APP_CFG_NON_CONN_ADV_INTERVAL_MS );
      //es_adv_timing_resume();   // xxxxx
       
    }
    
      err_code = app_timer_stop( adv_on_timer );
      APP_ERROR_CHECK(err_code);
      err_code = app_timer_start( adv_on_timer, ADV_ON_LASTTIME, NULL );
      APP_ERROR_CHECK(err_code);
  }
*/

}



// stop advertising when keep silent some time after last viberation
static void adv_on_timer_handler( void* p_context )
{
  UNUSED_PARAMETER( p_context );

  es_adv_timing_stop();
  es_adv_timing_start( 0xfffd );
    

  //adv_stop();

  //sd_ble_gap_adv_stop();

  adv_onoff = false;
  //led_Toggle( LED );
  //led_LitOff( LED, true );
  
}

void timers_Init( void )
{
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
                                          // APP_TIMER_MODE_SINGLE_SHOT, APP_TIMER_MODE_REPEATED
  err_code = app_timer_create( &adv_on_timer, APP_TIMER_MODE_SINGLE_SHOT, adv_on_timer_handler );
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_start( adv_on_timer, ADV_ON_LASTTIME, NULL );
  APP_ERROR_CHECK(err_code);

}





///////////////////////////////////////////////////////56
void led_Config( uint32_t led )
{
  nrf_gpio_cfg_output( led );
  nrf_gpio_pin_set( led );
}
void led_LitOff( uint32_t led, bool lit_off )
{
  if( lit_off == true )  nrf_gpio_pin_set( led );
  else if( lit_off == false )  nrf_gpio_pin_clear( led );
  else {;}
}
void led_Toggle( uint32_t led )
{
  nrf_gpio_pin_toggle( led );
}
///////////////////////////////////////////////////////56
/*
nrf_drv_uart_t  uart_driver_instance = NRF_DRV_UART_INSTANCE( UART0_INSTANCE_INDEX );
#define TESTUART_TX  9
#define TESTUART_RX  8
void uart0_config( void )
{
  ret_code_t  err_code;
  uint8_t thedata[16];

  //nrf_drv_uart_t  uart_driver_instance = NRF_DRV_UART_INSTANCE( UART0_INSTANCE_INDEX );
  nrf_drv_uart_config_t  config = NRF_DRV_UART_DEFAULT_CONFIG;

  config.pseltxd =  TESTUART_TX; 
  config.pselrxd =  TESTUART_RX;
  config.baudrate = UART_DEFAULT_CONFIG_BAUDRATE;  
  
  err_code = nrf_drv_uart_init( &uart_driver_instance,  &config,  NULL );
  APP_ERROR_CHECK( err_code );
  
  thedata[0] = 0x78;  thedata[1] = 0x79;  thedata[2] = 0x7a;  thedata[3] = 0x3e; 
  err_code = nrf_drv_uart_tx( &uart_driver_instance, thedata, 4 );    // xyz>
  APP_ERROR_CHECK( err_code );

}
void uart0_tx( int8_t ddata )
{
  ret_code_t  err_code;
  err_code = nrf_drv_uart_tx( &uart_driver_instance, &ddata, 1 );
  APP_ERROR_CHECK( err_code );
}   */
//////////////////////////////////////////////////////////////////67
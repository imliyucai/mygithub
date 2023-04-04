/*
 *   the head file of driver for LIS3DH and some related functions.         
 *   LIS3DH is a 3-axis accelerometer used as motion sensor.
 *
 *   liyucai  2022-12-24
 *
 */


#ifndef LIS3DH_H
#define LIS3DH_H

#ifdef __cplusplus
extern "C" {
#endif
//////////////////////////////////////////////////////////////////67

#define CHIPID    0x0f

#define CTRL_REG0    0x1e
#define CTRL_REG1    0x20
#define CTRL_REG2    0x21
#define CTRL_REG3    0x22
#define CTRL_REG4    0x23
#define CTRL_REG5    0x24
#define CTRL_REG6    0x25

#define REFERENCE     0x26
#define TEMP_CFG_REG   0x1f

#define STATUS_REG       0x27
#define STATUS_REG_AUX  0x07

#define OUT_X_L    0x28
#define OUT_X_H    0x29
#define OUT_Y_L    0x2a
#define OUT_Y_H    0x2b
#define OUT_Z_L    0x2c
#define OUT_Z_H    0x2d

#define OUT_ADC1_L   0x08
#define OUT_ADC1_H   0x09
#define OUT_ADC2_L   0x0a
#define OUT_ADC2_H   0x0b
#define OUT_ADC3_L   0x0c
#define OUT_ADC3_H   0x0d

#define FIFO_CTRL_REG    0x2e
#define FIFO_SRC_REG    0x2f

#define INT1_THS         0x32
#define INT1_DURATION    0x33
#define INT1_SRC         0x31
#define INT1_CFG         0x30

#define INT2_THS         0x36
#define INT2_DURATION    0x37
#define INT2_SRC         0x35
#define INT2_CFG         0x34

#define CLICK_CFG      0x38
#define CLICK_SRC      0x39
#define CLICK_THS      0x3a

#define TIME_LIMIT       0x3b
#define TIME_LATENCY     0x3c
#define TIME_WINDOW      0x3d

#define ACT_THS       0x3e
#define ACT_DUR       0x3f


/////////////////////////////////34
#define  LIS3DH_ADDR   0x19


///////////////////////////////////////////////////////56
#define TWI_INSTANCE_ID  0
#define TWI0M_SCL    7
#define TWI0M_SDA    30 

#define LIS3D_CS  29
#define LIS3D_SA0  31
#define LIS3D_INT1  28


#define LED   3

#include "nrf_drv_gpiote.h"
///////////////////////////////////////////////////////56
void twi0m_Init( void );
void twi0m_Stop( void );

void LIS3DH_ReadReg( uint8_t regaddr, uint8_t* pdata );
void LIS3DH_WriteReg( uint8_t regaddr, uint8_t ddata );
void LIS3DH_Config( void );

void int1_Interrupt_Config( void );
void int1_evt_handle( nrf_drv_gpiote_pin_t  pin, nrf_gpiote_polarity_t  action );

void timers_Init( void );

//void es_adv_timing_changeme( uint16_t adv_interval );


void led_Config( uint32_t led );
void led_LitOff( uint32_t led, bool lit_off );
void led_Toggle( uint32_t led );






//////////////////////////////////////////////////////////////////67
#ifdef __cplusplus
}
#endif

#endif // LIS3DH_H
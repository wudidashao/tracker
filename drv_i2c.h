/*******************************************************************************
 * drv_i2c.h
 *
 *  Created on: 2018Äê1ÔÂ23ÈÕ
 *      Author: vladimir
 *
 *****************************************************************************
*/

#ifndef DRV_I2C_H_
#define DRV_I2C_H_

#include <stdint.h>
#include <stdbool.h>

#define LOWBYTE(Data) (uint8_t)(0x00ff & Data)
#define HIGHBYTE(Data) (uint8_t)((0xff00 & Data)>>8)


//#define DELAY_LIMIT	(0x3FFF)
#define DELAY_LIMIT (0x1FFF)
//*****************************************************************************
//
// Definitions
//
//*****************************************************************************
#define DELAY_500_uS    4000    //sets cycles for 500us delay, based on 8MHz SMCLK
#define DELAY_1_MS      8000    //sets cycles for 1ms delay, based on 8MHz SMCLK
#define DELAY_10_MS     80100   //sets cycles for 10ms delay, based on 8MHz SMCLK
#define DELAY_100_MS    800000  //sets cycles for 100ms delay, based on 8MHz SMCLK

typedef enum logic
{
	FALSE,
	TRUE
}enum_logic_t;
//****************************************************************************
//
// Types
//
//****************************************************************************
typedef enum {
    eUSCI_IDLE = 0,
    eUSCI_SUCCESS = 0,
    eUSCI_BUSY = 1,
    eUSCI_NACK = 2,
    eUSCI_STOP,
    eUSCI_START
} eUSCI_status;
void I2CInitialise(void);
//void i2c_read_byte(uint8_t SLAVE_ADDRESS, uint8_t BYTE_COUNT);
//void i2c_write(uint8_t SLAVE_ADDRESS);
bool i2c_read_byte(uint8_t SLAVE_ADDRESS, uint8_t BYTE_COUNT);
bool i2c_write(uint8_t SLAVE_ADDRESS);
#endif /* DRV_I2C_H_ */

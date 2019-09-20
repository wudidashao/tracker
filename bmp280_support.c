 /*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bmp280_support.c
* Date: 2016/07/01
* Revision: 1.0.6
*
* Usage: Sensor Driver support file for BMP280 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bmp280.h"
#include "drv_i2c.h"
#include "bmp280_support.h"
#include <stdint.h>
extern uint8_t i2c_transmitCounter;// = 0;    //Variable to store transmit status for I2C
extern uint8_t *p_i2c_transmitData;     //Pointer to transmit data
extern uint8_t i2c_receiveCounter;      //Variable to store receive status for I2C
extern uint8_t *p_i2c_receivedData;     //Pointer to received data
#define BMP280_API
/*Enable the macro BMP280_API to use this support file */
/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
#ifdef BMP280_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of bytes of data to be read
 */
s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8 BMP280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of bytes of data to be read */
s8 BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 I2C_routine(void);
s8 SPI_routine(void);
#endif
/********************End of I2C/SPI function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP280_delay_msek(u32 msek);
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
uint32_t bmp280_data_readout_template(void);
/*----------------------------------------------------------------------------*
 *  struct bmp280_t parameters can be accessed by using bmp280
 *	bmp280_t having the following parameters
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bmp280_t bmp280;


#ifdef BMP280_API
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bmp280_t
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.dev_addr = BMP280_I2C_ADDRESS1;
	bmp280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}

/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bmp280_t
 *--------------------------------------------------------------------------*/
s8 SPI_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bmp280 the following structure parameter can be accessed
 *	Bus write function pointer: BMP280_WR_FUNC_PTR
 *	Bus read function pointer: BMP280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bmp280.bus_write = BMP280_SPI_bus_write;
	bmp280.bus_read = BMP280_SPI_bus_read;
	bmp280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}

/************** I2C/SPI buffer length ******/

#define	I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5
#define BUFFER_LENGTH	0xFF
#define	SPI_READ	0x80
#define SPI_WRITE	0x7F
#define BMP280_DATA_INDEX	1
#define BMP280_ADDRESS_INDEX	2

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bmp180.c
*
*-----------------------------------------------------------------------*/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value held in the array,
 *		which is written in the register
 *	\param cnt : The no of bytes of data to be written
 */
s8  BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	u8 array[cnt+1];
	u8 stringpos = BMP280_INIT_VALUE;
	array[BMP280_INIT_VALUE] = reg_addr;
	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos + BMP280_DATA_INDEX] = *(reg_data + stringpos);
	}
	p_i2c_transmitData = (uint8_t *)array;    //Transmit array start address
    i2c_transmitCounter = sizeof array;       //Load transmit byte counter
    i2c_write(dev_addr);
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMP280_INIT_VALUE
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated.Thus cnt+1 operation done in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of data to be read
 */
s8  BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BMP280_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN] = {BMP280_INIT_VALUE};
    u8 stringpos = {BMP280_INIT_VALUE};
    //array[BMP280_INIT_VALUE] = reg_addr;
    uint8_t HDC1000_Temp_Address[1] =
    {
     reg_addr    //Cap_MEAS2_LSB register address
    };

    /* Please take the below function as your reference
     * to read the data using I2C communication
     * add your I2C rad function here.
     * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
     * iError is an return value of SPI write function
     * Please select your valid return value
     * In the driver SUCCESS defined as BMP280_INIT_VALUE
     * and FAILURE defined as -1
     */
    p_i2c_transmitData = (uint8_t *)HDC1000_Temp_Address;       //Transmit array start address
    i2c_transmitCounter = sizeof HDC1000_Temp_Address;          //Load transmit byte counter
    i2c_write(dev_addr);
    __delay_cycles(4000);  //Delay 0.5ms for conversion
	p_i2c_receivedData = (uint8_t *)array;          //Receive array start address
	i2c_read_byte(dev_addr,cnt+1);  //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
      *(reg_data + stringpos) = array[stringpos];


	}
	return (s8)iError;
}

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is going to be read
 *	\param reg_data : This is the data read from the sensor, which is held in an array
 *	\param cnt : The no of data to be read
 */
s8  BMP280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError=BMP280_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN]={BUFFER_LENGTH};
	u8 stringpos;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address denotes the type of SPI data transfer, whether
	read/write (read as 1/write as 0)*/
	array[BMP280_INIT_VALUE] = reg_addr|SPI_READ;
	/*read routine is initiated by masking register address with 0x80*/
	/*
	* Please take the below function as your reference to
	* read the data using SPI communication
	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
	* add your SPI read function here
	* iError is an return value of SPI read function
	* Please select your valid return value
	* In the driver SUCCESS defined as 0
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. Thus cnt+1 operation done in the SPI read
	* and write string function
	* For more information please refer the SPI communication in data sheet
	*/
	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos+BMP280_DATA_INDEX];
	}

	return (s8)iError;
}


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void  BMP280_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
}
#endif

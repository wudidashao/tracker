/*
 * hdc1000.c
 *
 *  Created on: 2018Äê1ÔÂ23ÈÕ
 *      Author: vladimir
 *
 */

#include "driverlib.h"
#include "hdc1000.h"
#include "drv_i2c.h"

#define HDC1000_ADDRESS 0x40

//extern uint8_t f;   //Variable to store transmit status for I2C
extern uint8_t i2c_transmitCounter;// = 0;    //Variable to store transmit status for I2C
extern uint8_t *p_i2c_transmitData;     //Pointer to transmit data
extern uint8_t i2c_receiveCounter;      //Variable to store receive status for I2C
extern uint8_t *p_i2c_receivedData;     //Pointer to received data



//uint32_t q16temp;
//uint32_t q15temp;
//uint32_t q12temp;
//uint32_t q15humidity;
float       temperature;    //in degC
float       humidity;       //in %
uint16_t        Temperature;
uint16_t        Humidity;
extern uint16_t        Temperature_T;
extern uint16_t        Humidity_T;
void HDC1000_Init(void)
{
	uint8_t HDC1000_CDC_Config_SW_reset[3] =
	{
		0x02,	//CDC Config register address
		0x80,	//MSB of configuration (performs software reset of HDC1000)
		0x00	//LSB of configuration
	};

	uint8_t HDC1000_CDC_Config[3] =
	{
		0x02,	//CDC Config register address
		0x00,	//MSB of configuration (performs software reset of HDC1000)
		0x00	//LSB of configuration
	};

	p_i2c_transmitData = (uint8_t *)HDC1000_CDC_Config_SW_reset; 	//Transmit array start address
	i2c_transmitCounter = sizeof HDC1000_CDC_Config_SW_reset;		//Load transmit byte counter
	i2c_write(HDC1000_ADDRESS);

	__delay_cycles(DELAY_10_MS);	//Delay 10ms for HDC1000 reset

	p_i2c_transmitData = (uint8_t *)HDC1000_CDC_Config; 	//Transmit array start address
	i2c_transmitCounter = sizeof HDC1000_CDC_Config;		//Load transmit byte counter
	i2c_write(HDC1000_ADDRESS);
}

/*
 * Reads temp and HDC registers and converts them to IQ format
 */
int32_t HDC1000_TempRead(void)
{
//		int16_t temp = 0;
//		int32_t humidity = 0;
		uint8_t buf[4] = {0};

		uint8_t HDC1000_Temp_Address[1] =
		{
			0x00	//Cap_MEAS2_LSB register address
		};
		p_i2c_transmitData = (uint8_t *)HDC1000_Temp_Address;		//Transmit array start address
		i2c_transmitCounter = sizeof HDC1000_Temp_Address;    		//Load transmit byte counter
		i2c_write(HDC1000_ADDRESS);
		__delay_cycles(DELAY_10_MS+ 4*DELAY_1_MS);	//Delay 14ms for temp + humidity conversion
		p_i2c_receivedData = (uint8_t *)buf;		//Receive array start address
		i2c_read_byte(HDC1000_ADDRESS,sizeof buf);	//Read temperature data from HDC device
        __delay_cycles(DELAY_10_MS+ 4*DELAY_1_MS); //Delay 14ms for temp + humidity conversion
        Temperature   = ((((uint16_t)buf[0]) << 8U) & 0xff00) + (((uint16_t)buf[1]) & 0x00ff);
        Humidity    = ((((uint16_t)buf[2]) << 8U) & 0xff00) + (((uint16_t)buf[3]) & 0x00ff);
        temperature = (((float)(((uint32_t)Temperature) * 165))/65536) - 40;
        Temperature_T = (int)(temperature*10);
        humidity    = ((float)Humidity)*100/65536;
        Humidity_T    = (int)(humidity);
        __no_operation();//for debug
		return (temperature);								//Return temp. data
}

/*
 * Gets humdity value in IQ15 format, should be called after HDC1000_TempRead
 */
int32_t getHumidity(void)
{
	return humidity;
}

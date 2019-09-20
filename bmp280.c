/*
 ****************************************************************************
 * bmp280.c
 *
 *  Created on: 2018Äê1ÔÂ23ÈÕ
 *      Author: vladimir
 **************************************************************************/
#include "bmp280.h"
#include "driverlib.h"
#include "drv_i2c.h"
uint8_t Chip_ID;
uint8_t Ctrl_Measure;
uint8_t Config_Register;
uint8_t Tem_Register_M, Tem_Register_L, Tem_Register_XL;
uint8_t Pre_Register_M, Pre_Register_L, Pre_Register_XL;
uint8_t Cal[24];
int32_t pre;
int32_t bmp280RawPressure = 0;
unsigned short dig_T1;
unsigned short dig_T2;
unsigned short dig_T3;
unsigned short dig_P1;
unsigned short dig_P2;
unsigned short dig_P3;
unsigned short dig_P4;
unsigned short dig_P5;
unsigned short dig_P6;
unsigned short dig_P7;
unsigned short dig_P8;
unsigned short dig_P9;
extern uint8_t i2c_transmitCounter; // = 0;    //Variable to store transmit status for I2C
extern uint8_t *p_i2c_transmitData;     //Pointer to transmit data
extern uint8_t i2c_receiveCounter;    //Variable to store receive status for I2C
extern uint8_t *p_i2c_receivedData;     //Pointer to received data
static struct bmp280_t *p_bmp280; /**< pointer to BMP280 */
int64_t var1=0,var2=0,p=0;
/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	@param *bmp280 structure pointer.
 *
 *	@note While changing the parameter of the p_bmp280
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_init(struct bmp280_t *bmp280)
{
    /* variable used to return communication result*/
    BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    p_bmp280 = bmp280;/* assign BMP280 ptr */
    /*assign chip ID to the global structure*/
    p_bmp280->chip_id = read_chip_id();

    /* readout bmp280 calibration parameter structure */

    com_rslt += bmp280_get_calib_param();

    return com_rslt;
}
/*!
 *	@brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	@note 0xFA -> MSB -> bit from 0 to 7
 *	@note 0xFB -> LSB -> bit from 0 to 7
 *	@note 0xFC -> LSB -> bit from 4 to 7
 *
 *	@param v_uncomp_temperature_s32 : The uncompensated temperature.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_temperature(
        s32 *v_uncomp_temperature_s32)
{
    /* variable used to return communication result*/
    BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    /* Array holding the MSB and LSb value
     a_data_u8r[0] - Temperature MSB
     a_data_u8r[1] - Temperature LSB
     a_data_u8r[2] - Temperature LSB
     */
    u8 a_data_u8r[BMP280_TEMPERATURE_DATA_SIZE] = { BMP280_INIT_VALUE,
    BMP280_INIT_VALUE,
                                                    BMP280_INIT_VALUE };
    /* check the p_bmp280 structure pointer as NULL*/
    if (p_bmp280 == BMP280_NULL)
    {
        com_rslt = E_BMP280_NULL_PTR;
    }
    else
    {
        /* read temperature data */
        //com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
        //		BMP280_TEMPERATURE_MSB_REG, a_data_u8r,
        //		BMP280_TEMPERATURE_DATA_LENGTH);
        read_tem_register();
        a_data_u8r[0] = Tem_Register_M;
        a_data_u8r[1] = Tem_Register_L;
        a_data_u8r[2] = Tem_Register_XL;
        com_rslt = BMP280_INIT_VALUE;
        *v_uncomp_temperature_s32 =
                (s32) ((((u32) (a_data_u8r[BMP280_TEMPERATURE_MSB_DATA]))
                        << BMP280_SHIFT_BIT_POSITION_BY_12_BITS)
                        | (((u32) (a_data_u8r[BMP280_TEMPERATURE_LSB_DATA]))
                                << BMP280_SHIFT_BIT_POSITION_BY_04_BITS)
                        | ((u32) a_data_u8r[BMP280_TEMPERATURE_XLSB_DATA]
                                >> BMP280_SHIFT_BIT_POSITION_BY_04_BITS));
    }
    return com_rslt;
}

/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xF7 -> MSB -> bit from 0 to 7
 *	@note 0xF8 -> LSB -> bit from 0 to 7
 *	@note 0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	@param v_uncomp_pressure_s32 : The value of uncompensated pressure
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_pressure(
        s32 *v_uncomp_pressure_s32)
{
    /* variable used to return communication result*/
    BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    /* Array holding the MSB and LSb value
     a_data_u8[0] - Pressure MSB
     a_data_u8[1] - Pressure LSB
     a_data_u8[2] - Pressure LSB
     */
    u8 a_data_u8[BMP280_PRESSURE_DATA_SIZE] = { BMP280_INIT_VALUE,
    BMP280_INIT_VALUE,
                                                BMP280_INIT_VALUE };
    /* check the p_bmp280 structure pointer as NULL*/
    if (p_bmp280 == BMP280_NULL)
    {
        com_rslt = E_BMP280_NULL_PTR;
    }
    else
    {
        //com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
        //BMP280_PRESSURE_MSB_REG, a_data_u8,
        //BMP280_PRESSURE_DATA_LENGTH);
        read_presure_register();
        a_data_u8[0] = Pre_Register_M;
        a_data_u8[1] = Pre_Register_L;
        a_data_u8[2] = Pre_Register_XL;
        com_rslt = BMP280_INIT_VALUE;
        *v_uncomp_pressure_s32 =
                (s32) ((((u32) (a_data_u8[BMP280_PRESSURE_MSB_DATA]))
                        << BMP280_SHIFT_BIT_POSITION_BY_12_BITS)
                        | (((u32) (a_data_u8[BMP280_PRESSURE_LSB_DATA]))
                                << BMP280_SHIFT_BIT_POSITION_BY_04_BITS)
                        | ((u32) a_data_u8[BMP280_PRESSURE_XLSB_DATA]
                                >> BMP280_SHIFT_BIT_POSITION_BY_04_BITS));
    }
    return com_rslt;
}
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 uint32_t bmp280CompensateP(int32_t adcP)
{
    //int64_t var1=0,var2=0,p=0;
    var1=((int64_t)p_bmp280->calib_param.t_fine)-128000;
    var2=var1*var1*(int64_t)p_bmp280->calib_param.dig_P6;
    var2=var2+((var1*(int64_t)p_bmp280->calib_param.dig_P5)<<17);
    var2=var2+(((int64_t)p_bmp280->calib_param.dig_P4)<<35);
    var1=((var1*var1*(int64_t)p_bmp280->calib_param.dig_P3)>>8)+((var1*(int64_t)p_bmp280->calib_param.dig_P2)<<12);
    var1=(((((int64_t)1)<<47)+var1))*((int64_t)p_bmp280->calib_param.dig_P1)>>33;
//    if (var1==0)
//        return 0;
    p=1048576-adcP;
    p=(((p<<31)-var2)*3125)/var1;
    var1=(((int64_t)p_bmp280->calib_param.dig_P9)*(p>>13)*(p>>13))>>25;
    var2=(((int64_t)p_bmp280->calib_param.dig_P8)*p)>>19;
    p=((p+var1+var2)>>8)+(((int64_t)p_bmp280->calib_param.dig_P7)<<4);
    return(uint32_t)p;
}
/*!
 *	@brief Reads actual pressure from uncompensated pressure
 *	and returns the value in Pascal(Pa)
 *	@note Output value of "96386" equals 96386 Pa =
 *	963.86 hPa = 963.86 millibar
 *
 *
 *
 *
 *  @param  v_uncomp_pressure_s32: value of uncompensated pressure
 *
 *
 *
 *  @return Returns the Actual pressure out put as s32
 *
 */
uint32_t bmp280_compensate_pressure_int32(int32_t v_uncomp_pressure_s32)
{
    int32_t v_x1_u32r = BMP280_INIT_VALUE;
    int32_t v_x2_u32r = BMP280_INIT_VALUE;
    uint32_t v_pressure_u32 = BMP280_INIT_VALUE;
    v_pressure_u32 = 0;
    /* calculate x1*/
    v_x1_u32r = (((int32_t) p_bmp280->calib_param.t_fine)
            >> BMP280_SHIFT_BIT_POSITION_BY_01_BIT) - (int32_t) 64000;
    /* calculate x2*/
    v_x2_u32r = (((v_x1_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS)
            * (v_x1_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS))
            >> BMP280_SHIFT_BIT_POSITION_BY_11_BITS)
            * ((int32_t) p_bmp280->calib_param.dig_P6);
    v_x2_u32r = v_x2_u32r
            + ((v_x1_u32r * ((int32_t) p_bmp280->calib_param.dig_P5))
                    << BMP280_SHIFT_BIT_POSITION_BY_01_BIT);
    v_x2_u32r = (v_x2_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS)
            + (((int32_t) p_bmp280->calib_param.dig_P4)
                    << BMP280_SHIFT_BIT_POSITION_BY_16_BITS);
    /* calculate x1*/
    v_x1_u32r = (((p_bmp280->calib_param.dig_P3
            * (((v_x1_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS)
                    * (v_x1_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS))
                    >> BMP280_SHIFT_BIT_POSITION_BY_13_BITS))
            >> BMP280_SHIFT_BIT_POSITION_BY_03_BITS)
            + ((((int32_t) p_bmp280->calib_param.dig_P2) * v_x1_u32r)
                    >> BMP280_SHIFT_BIT_POSITION_BY_01_BIT))
            >> BMP280_SHIFT_BIT_POSITION_BY_18_BITS;
    v_x1_u32r = ((((32768 + v_x1_u32r))
            * ((int32_t) p_bmp280->calib_param.dig_P1))
            >> BMP280_SHIFT_BIT_POSITION_BY_15_BITS);
    /* calculate pressure*/
    v_pressure_u32 = (((uint32_t) (((int32_t) 1048576) - v_uncomp_pressure_s32)
            - (v_x2_u32r >> BMP280_SHIFT_BIT_POSITION_BY_12_BITS))) * 3125;
    /* check overflow*/
    if (v_pressure_u32 < 0x80000000)

        /* Avoid exception caused by division by zero */
        if (v_x1_u32r != BMP280_INIT_VALUE)
            v_pressure_u32 = (v_pressure_u32
                    << BMP280_SHIFT_BIT_POSITION_BY_01_BIT)
                    / ((uint32_t) v_x1_u32r);
        else
            return BMP280_INVALID_DATA;

    else

    /* Avoid exception caused by division by zero */
    if (v_x1_u32r != BMP280_INIT_VALUE)
        v_pressure_u32 = (v_pressure_u32 / (uint32_t) v_x1_u32r) * 2;
    else
        return BMP280_INVALID_DATA;

    /* calculate x1*/
    v_x1_u32r = (((int32_t) p_bmp280->calib_param.dig_P9)
            * ((int32_t) (((v_pressure_u32
                    >> BMP280_SHIFT_BIT_POSITION_BY_03_BITS)
                    * (v_pressure_u32 >> BMP280_SHIFT_BIT_POSITION_BY_03_BITS))
                    >> BMP280_SHIFT_BIT_POSITION_BY_13_BITS)))
            >> BMP280_SHIFT_BIT_POSITION_BY_12_BITS;
    /* calculate x2*/
    v_x2_u32r = (((int32_t) (v_pressure_u32 >>
    BMP280_SHIFT_BIT_POSITION_BY_02_BITS))
            * ((int32_t) p_bmp280->calib_param.dig_P8))
            >> BMP280_SHIFT_BIT_POSITION_BY_13_BITS;
    /* calculate true pressure*/
    v_pressure_u32 = (uint32_t) ((int32_t) v_pressure_u32
            + ((v_x1_u32r + v_x2_u32r + p_bmp280->calib_param.dig_P7)
                    >> BMP280_SHIFT_BIT_POSITION_BY_04_BITS));

    return v_pressure_u32;
}


/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address |   bit
 *------------|------------------|----------------
 *	dig_T1    |  0x88 and 0x89   | from 0 : 7 to 8: 15
 *	dig_T2    |  0x8A and 0x8B   | from 0 : 7 to 8: 15
 *	dig_T3    |  0x8C and 0x8D   | from 0 : 7 to 8: 15
 *	dig_P1    |  0x8E and 0x8F   | from 0 : 7 to 8: 15
 *	dig_P2    |  0x90 and 0x91   | from 0 : 7 to 8: 15
 *	dig_P3    |  0x92 and 0x93   | from 0 : 7 to 8: 15
 *	dig_P4    |  0x94 and 0x95   | from 0 : 7 to 8: 15
 *	dig_P5    |  0x96 and 0x97   | from 0 : 7 to 8: 15
 *	dig_P6    |  0x98 and 0x99   | from 0 : 7 to 8: 15
 *	dig_P7    |  0x9A and 0x9B   | from 0 : 7 to 8: 15
 *	dig_P8    |  0x9C and 0x9D   | from 0 : 7 to 8: 15
 *	dig_P9    |  0x9E and 0x9F   | from 0 : 7 to 8: 15
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_get_calib_param(void)
{
    /* variable used to return communication result*/
    BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    uint8_t i;
    u8 a_data_u8[BMP280_CALIB_DATA_SIZE] = { BMP280_INIT_VALUE,
    BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE,
                                             BMP280_INIT_VALUE };
    /* check the p_bmp280 structure pointer as NULL*/

    //com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
    //		BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG,
    //		a_data_u8,
    //		BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH);
    read_cal();
    for (i = 0; i < 24; i++)
    {
        a_data_u8[i] = Cal[i];
    }
    com_rslt = BMP280_INIT_VALUE;
    /* read calibration values*/
    p_bmp280->calib_param.dig_T1 = (u16) ((((u16) ((u8) a_data_u8[
    BMP280_TEMPERATURE_CALIB_DIG_T1_MSB]))
            << BMP280_SHIFT_BIT_POSITION_BY_08_BITS) | a_data_u8[
    BMP280_TEMPERATURE_CALIB_DIG_T1_LSB]);
    p_bmp280->calib_param.dig_T2 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_TEMPERATURE_CALIB_DIG_T2_MSB]))
            << BMP280_SHIFT_BIT_POSITION_BY_08_BITS) | a_data_u8[
    BMP280_TEMPERATURE_CALIB_DIG_T2_LSB]);
    p_bmp280->calib_param.dig_T3 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_TEMPERATURE_CALIB_DIG_T3_MSB]))
            << BMP280_SHIFT_BIT_POSITION_BY_08_BITS) | a_data_u8[
    BMP280_TEMPERATURE_CALIB_DIG_T3_LSB]);
    p_bmp280->calib_param.dig_P1 = (u16) ((((u16) ((u8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P1_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P1_LSB]);
    p_bmp280->calib_param.dig_P2 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P2_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P2_LSB]);
    p_bmp280->calib_param.dig_P3 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P3_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P3_LSB]);
    p_bmp280->calib_param.dig_P4 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P4_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P4_LSB]);
    p_bmp280->calib_param.dig_P5 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P5_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P5_LSB]);
    p_bmp280->calib_param.dig_P6 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P6_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P6_LSB]);
    p_bmp280->calib_param.dig_P7 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P7_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P7_LSB]);
    p_bmp280->calib_param.dig_P8 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P8_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P8_LSB]);
    p_bmp280->calib_param.dig_P9 = (s16) ((((s16) ((s8) a_data_u8[
    BMP280_PRESSURE_CALIB_DIG_P9_MSB])) << BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
            | a_data_u8[
            BMP280_PRESSURE_CALIB_DIG_P9_LSB]);

    return com_rslt;
}

BMP280_RETURN_FUNCTION_TYPE bmp280_get_forced_uncomp_pressure_temperature(
        s32 *v_uncomp_pressure_s32, s32 *v_uncomp_temperature_s32)
{
    /* variable used to return communication result*/
    BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = BMP280_INIT_VALUE;
    u8 v_waittime_u8 = BMP280_INIT_VALUE;
    /* check the p_bmp280 structure pointer as NULL*/
    if (p_bmp280 == BMP280_NULL)
    {
        com_rslt = E_BMP280_NULL_PTR;
    }
    else
    {
        /* read pressure and temperature*/
        v_data_u8 = (p_bmp280->oversamp_temperature
                << BMP280_SHIFT_BIT_POSITION_BY_05_BITS)
                + (p_bmp280->oversamp_pressure
                        << BMP280_SHIFT_BIT_POSITION_BY_02_BITS)
                + BMP280_FORCED_MODE;
        com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr,
                BMP280_CTRL_MEAS_REG, &v_data_u8,
                BMP280_GEN_READ_WRITE_DATA_LENGTH);
        bmp280_compute_wait_time(&v_waittime_u8);
        p_bmp280->delay_msec(v_waittime_u8);
        com_rslt += bmp280_read_uncomp_pressure_temperature(
                v_uncomp_pressure_s32, v_uncomp_temperature_s32);
    }
    return com_rslt;
}
/*!
 * @brief
 *	This API write the data to
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_write_register(u8 v_addr_u8, u8 *v_data_u8,
                                                  u8 v_len_u8)
{
    /* variable used to return communication result*/
    BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    /* check the p_bmp280 structure pointer as NULL*/
    if (p_bmp280 == BMP280_NULL)
    {
        com_rslt = E_BMP280_NULL_PTR;
    }
    else
    {
        com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr,
                v_addr_u8, v_data_u8, v_len_u8);
    }
    return com_rslt;
}
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMP280_RETURN_FUNCTION_TYPE bmp280_read_register(u8 v_addr_u8, u8 *v_data_u8,
                                                 u8 v_len_u8)
{
    /* variable used to return communication result*/
    BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    /* check the p_bmp280 structure pointer as NULL*/
    if (p_bmp280 == BMP280_NULL)
    {
        com_rslt = E_BMP280_NULL_PTR;
    }
    else
    {
        com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
                v_addr_u8, v_data_u8, v_len_u8);
    }
    return com_rslt;
}
/*
 * Reads chip_id D0
 */
int8_t read_chip_id(void)
{
//      int16_t temp = 0;
//      int32_t humidity = 0;
    uint8_t buf[2] = { 0 };

    uint8_t Register_Temp_Address[1] = { 0xD0  //Cap_MEAS2_LSB register address
            };
    p_i2c_transmitData = (uint8_t *) Register_Temp_Address; //Transmit array start address
    i2c_transmitCounter = sizeof Register_Temp_Address; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);  //Delay 14ms for temp + humidity conversion
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    Chip_ID = buf[0];
    //Temperature   = ((((uint16_t)buf[0]) << 8U) & 0xff00) + (((uint16_t)buf[1]) & 0x00ff);
    //Humidity    = ((((uint16_t)buf[2]) << 8U) & 0xff00) + (((uint16_t)buf[3]) & 0x00ff);
    //temperature = (((float)(((uint32_t)Temperature) * 165))/65536) - 40;
    //humidity    = ((float)Humidity)*100/65536;
    return (Chip_ID);                               //Return temp. data
}
/*
 * Reads ctrl_measure F4
 */
int8_t read_ctrl_measure(void)
{
//      int16_t temp = 0;
//      int32_t humidity = 0;
    uint8_t buf[2] = { 0 };

    uint8_t Register_Temp_Address[1] = { 0xF4  //Cap_MEAS2_LSB register address
            };
    p_i2c_transmitData = (uint8_t *) Register_Temp_Address; //Transmit array start address
    i2c_transmitCounter = sizeof Register_Temp_Address; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);  //Delay 14ms for temp + humidity conversion
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    Ctrl_Measure = buf[0];
    return (Ctrl_Measure);                               //Return temp. data
}
/*
 * Reads config_register F5
 */
int8_t read_config_register(void)
{
//      int16_t temp = 0;
//      int32_t humidity = 0;
    uint8_t buf[2] = { 0 };

    uint8_t Register_Temp_Address[1] = { 0xF5  //Cap_MEAS2_LSB register address
            };
    p_i2c_transmitData = (uint8_t *) Register_Temp_Address; //Transmit array start address
    i2c_transmitCounter = sizeof Register_Temp_Address; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);  //Delay 14ms for temp + humidity conversion
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    Config_Register = buf[0];
    //Temperature   = ((((uint16_t)buf[0]) << 8U) & 0xff00) + (((uint16_t)buf[1]) & 0x00ff);
    //Humidity    = ((((uint16_t)buf[2]) << 8U) & 0xff00) + (((uint16_t)buf[3]) & 0x00ff);
    //temperature = (((float)(((uint32_t)Temperature) * 165))/65536) - 40;
    //humidity    = ((float)Humidity)*100/65536;
    return (Config_Register);                               //Return temp. data
}
/*
 * Reads Tempeture_register FA
 */
int8_t read_tem_register(void)
{
//      int16_t temp = 0;
//      int32_t humidity = 0;
    uint8_t buf[4] = { 0 };

    uint8_t Register_Temp_Address[1] = { 0xFA  //Cap_MEAS2_LSB register address
            };
    p_i2c_transmitData = (uint8_t *) Register_Temp_Address; //Transmit array start address
    i2c_transmitCounter = sizeof Register_Temp_Address; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);  //Delay 14ms for temp + humidity conversion
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    Tem_Register_M = buf[0];
    Tem_Register_L = buf[1];
    Tem_Register_XL = buf[2];
    //Temperature   = ((((uint16_t)buf[0]) << 8U) & 0xff00) + (((uint16_t)buf[1]) & 0x00ff);
    //Humidity    = ((((uint16_t)buf[2]) << 8U) & 0xff00) + (((uint16_t)buf[3]) & 0x00ff);
    //temperature = (((float)(((uint32_t)Temperature) * 165))/65536) - 40;
    //humidity    = ((float)Humidity)*100/65536;
    return (Tem_Register_M);                               //Return temp. data
}
/*
 * Reads Presure_register F7
 */
int32_t read_presure_register(void)
{
    uint8_t buf[3] = { 0 };
    //int32_t temp = 0;
    uint8_t Register_Temp_Address[1] = { 0xF7  //Cap_MEAS2_LSB register address
            };
    p_i2c_transmitData = (uint8_t *) Register_Temp_Address; //Transmit array start address
    i2c_transmitCounter = sizeof Register_Temp_Address; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);  //Delay 14ms for temp + humidity conversion
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    Pre_Register_M = buf[0];
    Pre_Register_L = buf[1];
    Pre_Register_XL = buf[2];
    return (0);                               //Return temp. data
}
/*
 * Reads all F7
 */
uint32_t read_all(void)
{
//      int16_t temp = 0;
//      int32_t humidity = 0;
    uint8_t buf[7] = { 0 };
    //long adc_P;

    uint8_t Register_Temp_Address[1] = { 0xF7  //Cap_MEAS2_LSB register address
            };
    //p_bmp280->chip_id = read_chip_id();
    p_i2c_transmitData = (uint8_t *) Register_Temp_Address; //Transmit array start address
    i2c_transmitCounter = sizeof Register_Temp_Address; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);  //Delay 14ms for temp + humidity conversion
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    Pre_Register_M = buf[0];
    Pre_Register_L = buf[1];
    Pre_Register_XL = buf[2];
    Tem_Register_M = buf[3];
    Tem_Register_L = buf[4];
    Tem_Register_XL = buf[5];
    //adc_P = (int32_t)(((int32_t)Pre_Register_M << 12)|((int32_t)Pre_Register_L << 4)|((int32_t)Pre_Register_XL >> 4));

    //adc_T = (int32_t)(((int32_t)Tem_Register_M << 12)|((int32_t)Tem_Register_L << 4)|((int32_t)Tem_Register_XL >> 4));
    bmp280RawPressure = (int32_t) ((((uint32_t) Pre_Register_M) << 12)
            | (((uint32_t) Pre_Register_L) << 4)
            | (((uint32_t) Pre_Register_XL) >> 4));
    //pre = bmp280_compensate_pressure_double(adc_P);
    pre = bmp280CompensateP(bmp280RawPressure);
    return (pre);                               //Return temp. data
}/*
 * Reads all 88
 */
int8_t read_cal(void)
{
//      int16_t temp = 0;
//      int32_t humidity = 0;
    uint8_t buf[24] = { 0 };
    uint8_t i;

    uint8_t Register_Temp_Address[1] = { 0x88  //Cap_MEAS2_LSB register address
            };
    p_i2c_transmitData = (uint8_t *) Register_Temp_Address; //Transmit array start address
    i2c_transmitCounter = sizeof Register_Temp_Address; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);  //Delay 14ms for temp + humidity conversion
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS + DELAY_1_MS); //Delay 14ms for temp + humidity conversion
    for (i = 0; i < 24; i++)
    {
        Cal[i] = buf[i];
    }
    __delay_cycles(1600000);
    dig_T1 = (((short) buf[0] << 8) | (short) buf[1]);
    dig_T2 = (((short) buf[2] << 8) | (short) buf[3]);
    dig_T3 = (((short) buf[4] << 8) | (short) buf[5]);
    dig_P1 = (((short) buf[6] << 8) | (short) buf[7]);
    dig_P2 = (((short) buf[8] << 8) | (short) buf[9]);
    dig_P3 = (((short) buf[10] << 8) | (short) buf[11]);
    dig_P4 = (((short) buf[12] << 8) | (short) buf[13]);
    dig_P5 = (((short) buf[14] << 8) | (short) buf[15]);
    dig_P6 = (((short) buf[16] << 8) | (short) buf[17]);
    dig_P7 = (((short) buf[18] << 8) | (short) buf[19]);
    dig_P8 = (((short) buf[20] << 8) | (short) buf[21]);
    dig_P9 = (((short) buf[22] << 8) | (short) buf[23]);

    return (Pre_Register_M);                               //Return temp. data
}
void bmp280_Init_0(void)
{
    // unsigned char temp = 0;
    uint8_t Register_reset_Address1[2] = { 0xe0, 0xb6 };
    uint8_t Register_reset_Address2[2] = { 0xf4, 0xff };
    uint8_t Register_reset_Address3[2] = { 0xf5, 0x00 };
    p_bmp280->chip_id = read_chip_id();
    bmp280_get_calib_param();
    p_i2c_transmitData = (uint8_t *) Register_reset_Address1; //Transmit array start address
    i2c_transmitCounter = sizeof Register_reset_Address1; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);
    p_i2c_transmitData = (uint8_t *) Register_reset_Address2; //Transmit array start address
    i2c_transmitCounter = sizeof Register_reset_Address2; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);
    p_i2c_transmitData = (uint8_t *) Register_reset_Address3; //Transmit array start address
    i2c_transmitCounter = sizeof Register_reset_Address3; //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    //bmp280_get_calib_param();
//    read_cal();
//
//    dig_T2 = bmp280_MultipleReadTwo(0x8A);
//    dig_T3 = bmp280_MultipleReadTwo(0x8C);
//    dig_P1 = bmp280_MultipleReadTwo(0x8E);
//    dig_P2 = bmp280_MultipleReadTwo(0x90);
//    dig_P3 = bmp280_MultipleReadTwo(0x92);
//    dig_P4 = bmp280_MultipleReadTwo(0x94);
//    dig_P5 = bmp280_MultipleReadTwo(0x96);
//    dig_P6 = bmp280_MultipleReadTwo(0x98);
//    dig_P7 = bmp280_MultipleReadTwo(0x9A);
//    dig_P8 = bmp280_MultipleReadTwo(0x9C);
//    dig_P9 = bmp280_MultipleReadTwo(0x9E);
    __delay_cycles(1600000);
}
uint8_t bmp280_ReadByte(uint8_t addr)
{
    uint8_t temp;
    uint8_t tmp[1] = { addr };
    uint8_t buf[1] = { 0 };
    p_i2c_transmitData = (uint8_t *) tmp[1];      //Transmit array start address
    i2c_transmitCounter = sizeof tmp;          //Load transmit byte counter
    i2c_write(BMP280_I2C_ADDRESS1);
    __delay_cycles(DELAY_1_MS);
    p_i2c_receivedData = (uint8_t *) buf;        //Receive array start address
    i2c_read_byte(BMP280_I2C_ADDRESS1, sizeof buf); //Read temperature data from HDC device
    __delay_cycles(DELAY_1_MS + DELAY_1_MS);
    temp = buf[0];
    return temp;
}
short bmp280_MultipleReadTwo(uint8_t addr)
{
    uint8_t msb, lsb;
    short temp = 0;
    lsb = bmp280_ReadByte(addr);
    msb = bmp280_ReadByte(addr + 1);

    temp = (short) msb << 8;
    temp |= (short) lsb;

    return temp;
}
long bmp280_MultipleReadThree(uint8_t addr)
{
    uint8_t msb, lsb, xlsb;
    long temp = 0;
    msb = bmp280_ReadByte(addr);
    lsb = bmp280_ReadByte(addr + 1);
    xlsb = bmp280_ReadByte(addr + 2);

    temp = (long) (((unsigned long) msb << 12) | ((unsigned long) lsb << 4)
            | ((unsigned long) xlsb >> 4));

    return temp;
}
long bmp280_GetValue(void)
{
    long adc_T;
    long adc_P;
    long var1, var2, t_fine, p;  //T;

    adc_T = bmp280_MultipleReadThree(BMP280_TEMPERATURE_MSB_REG);
    adc_P = bmp280_MultipleReadThree(BMP280_PRESSURE_MSB_REG);

    if (adc_P == 0)
    {
        return 0;
    }

    //Temperature
    var1 = (((double) adc_T) / 16384.0 - ((double) dig_T1) / 1024.0)
            * ((double) dig_T2);
    var2 = ((((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)
            * (((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0))
            * ((double) dig_T3);

    t_fine = (unsigned long) (var1 + var2);

    //T = (var1+var2)/5120.0;

    var1 = ((double) t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double) dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
    var1 = (((double) dig_P3) * var1 * var1 / 524288.0
            + ((double) dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);
    p = 1048576.0 - (double) adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double) dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double) dig_P7)) / 16.0;

    return p;
}

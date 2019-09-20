/*
 * config.h
 *
 *  Created on: 2018年7月16日
 *      Author: zlzhang
 *****************************************************************************
 */

#include "FRAMLogMode.h"
#include "usca_uart.h"
#include "driverlib.h"
/*
 * FRAM Array reserved to store data memory for Temperature and Voltage
 * Stores up to 10000 datapoints
 *
 */
#if defined(__IAR_SYSTEMS_ICC__)
#pragma location = 0x9000
__no_init uint16_t sensordata[12289];
#elif defined(__TI_COMPILER_VERSION__)
//#pragma PERSISTENT(config_data)
//uint8_t config_data[20] = {0};
#pragma PERSISTENT(sensordata)
uint8_t sensordata[40960] = { 0 };
#pragma PERSISTENT(alarmdata)
uint8_t alarmdata[53] = { 0 };
#pragma PERSISTENT(dataIndex)
uint16_t dataIndex = 0;
#pragma PERSISTENT(sensordata_header)
uint8_t sensordata_header[2] = { 0x24, 0x24 };
#pragma PERSISTENT(sensordata_length)
uint8_t sensordata_length[2] = { 0 };
#endif

// FRAM Array reserved to store FRAM Log Mode starting Time Stamp
#if defined(__IAR_SYSTEMS_ICC__)
#pragma location = 0xF002
__no_init uint8_t timeStamp[6];
#elif defined(__TI_COMPILER_VERSION__)
// 13 bytes Unix time stamp received from PC (milliseconds elapsed since 1 January 1970 00:00:00 UTC up until now)
#pragma PERSISTENT(timeStamp)
uint8_t timeStamp[48] = { 0 };
#pragma PERSISTENT(count_capture)
int16_t count_capture = 0;
#pragma PERSISTENT(count_tx)
int16_t count_tx = 0;
#pragma PERSISTENT(jiaoyan)
uint8_t jiaoyan = 0;
#pragma PERSISTENT(flag_capture)
uint8_t flag_capture = 0;
#pragma PERSISTENT(flag_tx)
uint8_t flag_tx = 0;
#endif
int16_t count_tx_tmp = 0, count_length = 0;
uint8_t data_temp;
extern uint8_t gprs_ok;
extern uint8_t sensors_data[];
extern uint8_t alarm_data[];
extern int mode;
extern uint8_t AT_flag;
uint8_t command_ok_flag = 0;
void framLog_fifo(uint8_t *ptr)
{
    //__disable_interrupt();
    uint8_t i;
    if (flag_capture == flag_tx)
    {
        for (i = 0; i < 48; i++)
        {
            sensordata[(count_capture * 48) + i] = *ptr++;

        }
        count_capture++;
//        sensordata_length[0] = dataIndex >> 8;
//        sensordata_length[1] = dataIndex ;
        if (count_capture >= 853)
        {
            count_capture = 0;
            //count_tx = 0;
            if (flag_capture == 0)
            {
                flag_capture = 1;
            }
            else
            {
                flag_capture = 0;
            }
        }
    }
    else
    {
        if (count_capture < count_tx)
        {
            for (i = 0; i < 48; i++)
            {
                sensordata[(count_capture * 48) + i] = *ptr++;

            }
            count_capture++;
        }
        else if (count_capture == count_tx)
        {
            for (i = 0; i < 48; i++)
            {
                sensordata[(count_capture * 48) + i] = *ptr++;

            }
            count_capture++;
            count_tx++;
            if (count_capture >= 853)
            {
                count_capture = 0;
                count_tx = 0;
                if (flag_capture == 0)
                {
                    flag_capture = 1;
                    flag_tx = 0;
                }
                else
                {
                    flag_capture = 0;
                    flag_tx = 1;
                }
            }
        }
    }
}
void framLog(uint8_t *ptr)
{
    //__disable_interrupt();
//    int i;
    uint8_t j;
//    if (dataIndex >= 48)
//    {
//        for (j = 0; j < 53; j++)
//        {
//            // clear buf
//            alarmdata[j] = 0x00;
//
//        }
        dataIndex = 0;
//    }
    for (j = 0; j < 48; j++)
    {
        alarmdata[dataIndex + 4] = *ptr++;
        dataIndex++;
    }
    //__enable_interrupt();
}
/*
 * Send FRAMLogMode starting TimeStamp through UART
 */
void sendTimeStampFRAM()
{
    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0,
    GPIO_SECONDARY_MODULE_FUNCTION);
    __delay_cycles(900000);
    int i;
    for (i = 0; i < 13; i++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, timeStamp[i]);
    }

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
}
/*
 * Send SDCard temperature and voltage data through UART
 */
void sendDataFRAM()
{
    uint16_t i;
    alarmdata[0] = 0x24;
    alarmdata[1] = 0x24;
    alarmdata[2] = dataIndex >> 8;
    alarmdata[3] = dataIndex;
    for (i = 0; i < dataIndex + 4; i++)
        data_temp += alarmdata[i];
    alarmdata[dataIndex + 4] = data_temp;
    data_temp = 0;
    Init_Gprs();
    if (gprs_ok)
    {
        Connect_Server(); //设置GPRS参数
        if (AT_flag)
        {
            for (i = 0; i < dataIndex + 5; i++)
            {
                // Send logged Temperature Sensor data
                EUSCI_A_UART_transmitData(EUSCI_A1_BASE, alarmdata[i]);
            }
            gprs_ok = 0;
            AT_flag = 0;
            _delay_cycles(40000000);
            command_analysis();
            if (command_ok_flag)
            {
                command_ok_flag = 0;
                dataIndex = 0;
                __delay_cycles(4000);
            }
        }
    }

}
/*
 * Send SDCard temperature and voltage data through UART
 */
void sendDataFRAM_fifo(void)
{
    uint8_t i;
    Init_Gprs();
    if (gprs_ok)
    {
        gprs_ok = 0;
        Connect_Server(); //设置GPRS参数
        if (AT_flag)
//        while (AT_flag)
        {
            AT_flag = 0;
            if (flag_capture == flag_tx)
            {
                count_tx_tmp = count_tx + 20;
                if (count_tx_tmp > count_capture)
                {
                    if ((count_capture - 1) >= count_tx)
                    {
                        count_length = (count_capture - count_tx) * 48;
                        if (count_length <= 0x00)
                        {
#if send_none_frame
                            framLog(alarm_data);
                            for (i = 0; i < dataIndex + 5; i++)
                            {
                                // Send logged Temperature Sensor data
                                EUSCI_A_UART_transmitData(EUSCI_A1_BASE, alarmdata[i]);
                            }
#endif
                            AT_flag = 0;
                            __no_operation();    //for debug
                            //break;
                        }
                        else
                        {
                            send_data_fifo();
                            if (command_ok_flag)
                            {
                                command_ok_flag = 0;
                                count_tx = count_capture;
                                //AT_flag = 0;
                                //count_tx += count_length/48;
                                __delay_cycles(4000);
                            }
                            else
                            {
                                AT_flag = 0;
                                __no_operation();    //for debug
                                //break;
                            }
                        }
                        //temp++;

                    }
                }
                else
                {
                    if ((count_capture - 1) > count_tx)
                    {
                        count_length = (count_tx_tmp - count_tx) * 48;
                        if (count_length <= 0x00)
                        {
                            AT_flag = 0;
                            __no_operation();    //for debug
                            //break;
                        }
                        else
                        {
                            send_data_fifo();
                            if (command_ok_flag)
                            {
                                command_ok_flag = 0;
                                count_tx = count_tx_tmp;
                                //count_tx_tmp = count_tx + 20;
                                __delay_cycles(4000);
                            }
                            else
                            {
                                AT_flag = 0;
                                __no_operation();    //for debug
                                //break;
                            }
                        }
                        //send_data_fifo();

                    }

                }
            }
            else
            {
                count_tx_tmp = count_tx + 20;
                if (count_tx_tmp > 852)
                {
                    //if ((count_capture - 1) > 0)
//                    while ((count_capture - 1) > count_tx)
//                    {
                    count_length = (853 - count_tx) * 48;
                    if (count_length <= 0x00)
                    {
                        count_tx = 0;
                        if (flag_tx == 0)
                        {
                            flag_tx = 1;
                        }
                        else
                        {
                            flag_tx = 0;
                        }
                        AT_flag = 0;
                        __no_operation();    //for debug
                        //break;
                    }
                    else
                    {
                        send_data_fifo();
                        if (command_ok_flag)
                        {
                            command_ok_flag = 0;
                            count_tx = 0;
                            if (flag_tx == 0)
                            {
                                flag_tx = 1;
                            }
                            else
                            {
                                flag_tx = 0;
                            }
                            __delay_cycles(4000);
                        }
                        else
                        {
                            AT_flag = 0;
                            __no_operation();    //for debug
                           // break;
                        }
                    }
                    //send_data_fifo();

//                    }
                }
                else
                {
                    //if ((count_capture - 1) > 0)
//                    while ((count_capture - 1) > count_tx)
//                    {
                    count_length = (count_tx_tmp - count_tx) * 48;
                    if (count_length <= 0x00)
                    {
                        AT_flag = 0;
                        __no_operation();    //for debug
                        //break;
                    }
                    else
                    {
                        send_data_fifo();
                        if (command_ok_flag)
                        {
                            command_ok_flag = 0;
                            count_tx = count_tx_tmp;
                            //count_tx += count_length/48;
                            __delay_cycles(4000);
                        }
                        else
                        {
                            AT_flag = 0;
                            __no_operation();    //for debug
                            //break;
                        }
                    }
                    //send_data_fifo();
//                    }
                }

            }
        }
    }

}
void send_data_fifo(void)
{
    uint16_t i;
    sensordata_length[0] = count_length >> 8;
    sensordata_length[1] = count_length;
    jiaoyan = (sensordata_header[0] + sensordata_header[1]
            + sensordata_length[0] + sensordata_length[1]);
    for (i = (count_tx * 48); i < ((count_tx * 48) + count_length); i++)
        jiaoyan += sensordata[i];
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, sensordata_header[0]);
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, sensordata_header[1]);
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, sensordata_length[0]);
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, sensordata_length[1]);
    for (i = (count_tx * 48); i < ((count_tx * 48) + count_length); i++)
    {
        // Send logged Temperature Sensor data
        EUSCI_A_UART_transmitData(EUSCI_A1_BASE, sensordata[i]);
    }
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, jiaoyan);
    jiaoyan = 0;
    _delay_cycles(40000000);
    command_analysis();
}
void storeTimeStampFRAM()
{
    int i;

    for (i = 0; i < 13; i++)
    {
        __bis_SR_register(LPM3_bits | GIE); // Enter LPM3. Delay for Ref to settle.
        __no_operation();

        timeStamp[i] = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
    }
}

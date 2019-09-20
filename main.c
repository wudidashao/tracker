/*
 * main.c
 *
 *  Created on: 2018年1月05日
 *      Author: vladimir
 */
#include <math.h>
#include <stdio.h>
#include "driverlib.h"
#include "FRAMLogMode.h"
#include "usca_uart.h"
#include "gpio.h"
//#include "i2c_driver.h"
#include "drv_i2c.h"
#include "hdc1000.h"
#include "bmp280_support.h"
#include "bmp280.h"
#include "OPT3001.h"
#include "adc12b.h"
#include "config.h"
#pragma PERSISTENT(time_Stamp)
uint8_t time_Stamp[3] = { 0x00, 0x30, 0x30 };
#pragma PERSISTENT(sms_data)
uint8_t sms_data[48] = { 0xEE, 0x90, 0x4E, 0x31, 0x16, 0x45, 0x87, 0x45, 0x01,
                         0x20, 0x39, 0x91, 0x48, 0x00, 0x00, 0x61, 0x01, 0x41,
                         0x78, 0x02, 0x54, 0xFE, 0x70, 0x00, 0x00, 0x08, 0xC1,
                         0xFE, 0xA4, 0x10, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x42, 0x03, 0x02, 0x17, 0x03, 0x07, 0x18,
                         UUID_H,
                         UUID_L, 0x00 };       //EE 90
uint8_t sms_data_bk[96] = { 0 };
uint8_t *str_sms;
int main(void)
{
    uint16_t i;
    // Stop Watchdog timer
    WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);     // Stop WDT
    //WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    //WDTCTL = WDT_ARST_1000;    //1000ms
    // Board initializations
    Init_GPIO();
    Init_Clock();
    Init_RTC_C();
    __no_operation();                //for debug
#if 0
    gps_data_process();
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);    //open sensor power
    __delay_cycles(40000);

    I2CInitialise();//I2C start
    HDC1000_Init();
    temperatureData = HDC1000_TempRead();//HDC1080
    bmp280_Init_0();
    g_u32ActualPress = read_all();
    g_u32ActualPress = g_u32ActualPress/256;
    //    StartLuxMeasurement();
    //    __delay_cycles(1000000);
    //    luxData = GetLuxValue();//OPT3001
    if (Temperature_T > 0)
    {
        Temperature_T_temp = Temperature_T;
        Temperature_T_temp = (Temperature_T_temp | 0x8000);
    }
    else
    {
        Temperature_T_temp = abs(Temperature_T);
    }
    sensors_data[21] = ((Temperature_T_temp & 0xff00) >> 8);  //temperature
    sensors_data[22] = (Temperature_T_temp & 0x00ff);
    sensors_data[23] = (Humidity_T >> 8);
    sensors_data[24] = Humidity_T;
    sensors_data[31] = ((g_u32ActualPress & 0xff000000) >> 24);//actualpress
    sensors_data[32] = ((g_u32ActualPress & 0xff0000) >> 16);
    sensors_data[33] = ((g_u32ActualPress & 0xff00) >> 8);
    sensors_data[34] = g_u32ActualPress;
//    lux = (int)(uint16_t)luxData;
//    sensors_data[25]= (lux>>8);//humidity
//    sensors_data[26]= lux;
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);//close sensor power
    for(i=0;i<30;i++) all_data_fram();
    gprs_data_process();
#endif
#if 1
    adc12_b_initialise();    //voltage
    adc12b_start_conversion();
    voltage_T = ((voltage / 4096) * 1.2) * 6 * 1000;
    if (voltage_T < voltage_threshold_tmp)
    {
        send_frame_alarm = 1;
        alarm_data[3] = 0x01;
    }
    alarm_data[29] = (voltage_T >> 8);
    alarm_data[30] = voltage_T;
    alarm_data[2] = 0x01;    //复位标志
    reset_alarm_count++;
    alarm_data[44] = reset_alarm_count;
    for (i = 0; i < 47; i++)
        alarm_data[47] ^= alarm_data[i];
    gprs_data_process_alarm();
//    all_data_fram();
//    gprs_data_process();
//    RTC_C_holdClock(RTC_C_BASE);
    Init_RTC_C_reset_gprs();
#endif
    __no_operation();    //for debug
    while (1)
    {
        if (tle4913_flag)
        {
            __delay_cycles(80);    //delay 10us wait for dco ok
            Timer_A_stop(TIMER_A4_BASE); //disable timerA4
            Timer_A_clear(TIMER_A4_BASE);
//            RTC_C_holdClock(RTC_C_BASE);
            tle4913_flag = 0;
            __bis_SR_register(LPM4_bits + GIE);
        }
        else
        {
            if (data_ok_flag)
            {
                if (Send_time_offset_tmp > 0)
                {
                    for (i = 0; i < Send_time_offset_tmp; i++)
                    {
                        __delay_cycles(8000000);      //delay 1s wait for dco ok
                    }
                }
                if (send_frame_alarm)
                {
                    gprs_data_process_alarm();
                }
                else
                {
                    gprs_data_process();
                }
//                RTC_C_holdClock(RTC_C_BASE);
                Init_RTC_C_reset_gprs();
            }
            else if (timer_flag_h)
            {
                __delay_cycles(80);            //delay 10us wait for dco ok
                timer_flag_h = 0;
                sensor_data_process();
                if (send_frame_alarm)
                {
                    framLog(alarm_data);
                }
                else
                {
                    gps_data_process();
                    all_data_fram();
                }
                __no_operation();                //for debug
            }
            if ((timer_flag_h == 0) && (data_ok_flag == 0))
            {
                Low_Init_GPIO();
                __bis_SR_register(LPM3_bits + GIE);
            }

        }

    }
}
void Init_RTC_C_reset_gprs(void)
{
    uint8_t i;
    uint8_t tmp;
    switch (Send_times_of_day)
    {

        case 0x00:
        times_0 = Send_time_reference;
        hoursAlarm_value = times_0;
        break;
        case 0x01:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);

        for(i=0;i<2;i++)
        {
            times_1[i] = ((i*12)+time_hour_temp);
            tmp = hourchange24(times_1[i]);
            times_1[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_1,2);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        if (time_hour_temp>=times_1[0])
        {
            time_temp = times_1[1];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        else
        {
            time_temp = times_1[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        break;
        case 0x02:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<4;i++)
        {
            times_2[i] = ((i*6)+time_hour_temp);
            tmp = hourchange24(times_2[i]);
            times_2[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_2,4);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<3;i++)
        {
            if ((time_hour_temp>=times_2[i]) && (time_hour_temp<=times_2[i+1]))
            {
                time_temp = times_2[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_2[3])||(time_hour_temp>=times_2[0]))
        {
            time_temp = times_2[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x03:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<6;i++)
        {
            times_3[i] = ((i*4)+time_hour_temp);
            tmp = hourchange24(times_3[i]);
            times_3[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_3,6);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<5;i++)
        {
            if ((time_hour_temp>=times_3[i]) && (time_hour_temp<=times_3[i+1]))
            {
                time_temp = times_3[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_3[5])||(time_hour_temp<times_3[0]))
        {
            time_temp = times_3[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x04:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<12;i++)
        {
            times_4[i] = ((i*2)+time_hour_temp);
            tmp = hourchange24(times_4[i]);
            times_4[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_4,12);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<11;i++)
        {
            if ((time_hour_temp>=times_4[i]) && (time_hour_temp<=times_4[i+1]))
            {
                time_temp = times_4[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_4[11])||(time_hour_temp<times_4[0]))
        {
            time_temp = times_4[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x05:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        __no_operation();                //for debug
        for(i=0;i<24;i++)
        {
            times_5[i] = i;
            //times_5[i] = hourchange24(times_5[i]);
        }
        tmp0 = time_Stamp[0];
        time_hour_temp = itoa_bcd_dec(tmp0);
        __no_operation();                //for debug
        for(i=0;i<23;i++)
        {
            if ((time_hour_temp>=times_5[i]) && (time_hour_temp<=times_5[i+1]))
            {
                time_temp = times_5[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if(time_hour_temp>=times_5[23])
        {
            time_temp = times_5[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        __no_operation();                //for debug
        break;
        default:
        Send_times_of_day = 0x00;
        hoursAlarm_value = Send_time_reference;
        break;

    }
//    currentTime.Seconds = time_Stamp[2];
//    currentTime.Minutes = time_Stamp[1];
//    currentTime.Hours = time_Stamp[0];
//    currentTime.DayOfWeek = 0x03;
//    currentTime.DayOfMonth = 0x10;
//    currentTime.Month = 0x10;
//    currentTime.Year = 0x2018;
//    RTC_C_initCalendar(RTC_C_BASE, &currentTime,
//    RTC_C_FORMAT_BCD);
    //Setup Calendar Alarm for 5:00pm on the 5th day of the week.
    //Note: Does not specify day of the week.
    RTC_C_configureCalendarAlarmParam param = { 0 };
    param.minutesAlarm = 0x00;
    param.hoursAlarm = hoursAlarm_value;
    param.dayOfWeekAlarm = RTC_C_ALARMCONDITION_OFF;
    param.dayOfMonthAlarm = RTC_C_ALARMCONDITION_OFF;
    RTC_C_configureCalendarAlarm(RTC_C_BASE, &param);

    //Specify an interrupt to assert every minute
    if (intergral_point_flag == 0x01)
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_HOURCHANGE);    //RTC_C_CALENDAREVENT_MINUTECHANGE
    }
    else
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_MINUTECHANGE);    //RTC_C_CALENDAREVENT_HOURCHANGE
    }
    //Enable interrupt for RTC Ready Status, which asserts when the RTC
    //Calendar registers are ready to read.
    //Also, enable interrupts for the Calendar alarm and Calendar event.
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIE + RTCAIE);    //RTCRDYIE +
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIFG + RTCAIFG);    //RTCRDYIFG +
    RTC_C_disableInterrupt(RTC_C_BASE,
    RTC_C_PRESCALE_TIMER0_INTERRUPT);
    //hoursAlarm_value = Send_time_reference;
    //Start RTC Clock
    RTC_C_startClock(RTC_C_BASE);
}

void Init_RTC_C_gps_first(void)
{
    uint8_t i;
    uint8_t tmp;
    switch (Send_times_of_day)
    {

        case 0x00:
        times_0 = Send_time_reference;
        hoursAlarm_value = times_0;
        break;
        case 0x01:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<2;i++)
        {
            times_1[i] = ((i*12)+time_hour_temp);
            tmp = hourchange24(times_1[i]);
            times_1[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_1,2);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        if (time_hour_temp>=times_1[0])
        {
            time_temp = times_1[1];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        else
        {
            time_temp = times_1[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        break;
        case 0x02:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<4;i++)
        {
            times_2[i] = ((i*6)+time_hour_temp);
            tmp = hourchange24(times_2[i]);
            times_2[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_2,4);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<3;i++)
        {
            if ((time_hour_temp>=times_2[i]) && (time_hour_temp<=times_2[i+1]))
            {
                time_temp = times_2[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_2[3])||(time_hour_temp>=times_2[0]))
        {
            time_temp = times_2[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x03:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<6;i++)
        {
            times_3[i] = ((i*4)+time_hour_temp);
            tmp = hourchange24(times_3[i]);
            times_3[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_3,6);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<5;i++)
        {
            if ((time_hour_temp>=times_3[i]) && (time_hour_temp<=times_3[i+1]))
            {
                time_temp = times_3[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_3[5])||(time_hour_temp<times_3[0]))
        {
            time_temp = times_3[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x04:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<12;i++)
        {
            times_4[i] = ((i*2)+time_hour_temp);
            tmp = hourchange24(times_4[i]);
            times_4[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_4,12);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<11;i++)
        {
            if ((time_hour_temp>=times_4[i]) && (time_hour_temp<=times_4[i+1]))
            {
                time_temp = times_4[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_4[11])||(time_hour_temp<times_4[0]))
        {
            time_temp = times_4[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x05:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        __no_operation();                //for debug
        for(i=0;i<24;i++)
        {
            times_5[i] = i;
            //times_5[i] = hourchange24(times_5[i]);
        }
        tmp0 = time_Stamp[0];
        time_hour_temp = itoa_bcd_dec(tmp0);
        __no_operation();                //for debug
        for(i=0;i<23;i++)
        {
            if ((time_hour_temp>=times_5[i]) && (time_hour_temp<=times_5[i+1]))
            {
                time_temp = times_5[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if(time_hour_temp>=times_5[23])
        {
            time_temp = times_5[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        __no_operation();                //for debug
        break;
        default:
        Send_times_of_day = 0x00;
        hoursAlarm_value = Send_time_reference;
        //hoursAlarm_value = Send_time_reference;
        break;

    }
    //minute_count = 0;
    //Setup Current Time for Calendar
    currentTime.Seconds = time_Stamp[2];
    currentTime.Minutes = time_Stamp[1];
    currentTime.Hours = time_Stamp[0];
    currentTime.DayOfWeek = 0x03;
    currentTime.DayOfMonth = 0x10;
    currentTime.Month = 0x10;
    currentTime.Year = 0x2018;

    //Initialize Calendar Mode of RTC
    /*
     * Base Address of the RTC_A
     * Pass in current time, intialized above
     * Use BCD as Calendar Register Format
     */
    RTC_C_initCalendar(RTC_C_BASE, &currentTime,
    RTC_C_FORMAT_BCD);
    //hoursAlarm_value=hourchangebcd_fifo(Send_time_reference,Send_times_of_day);
    //Setup Calendar Alarm for 5:00pm on the 5th day of the week.
    //Note: Does not specify day of the week.
    RTC_C_configureCalendarAlarmParam param = { 0 };
    param.minutesAlarm = 0x00;
    param.hoursAlarm = hoursAlarm_value;    //Send_time_reference;
    param.dayOfWeekAlarm = RTC_C_ALARMCONDITION_OFF;
    param.dayOfMonthAlarm = RTC_C_ALARMCONDITION_OFF;
    RTC_C_configureCalendarAlarm(RTC_C_BASE, &param);

//    //Specify an interrupt to assert every minute
    if (intergral_point_flag == 0x01)
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_HOURCHANGE);    //RTC_C_CALENDAREVENT_MINUTECHANGE
    }
    else
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_MINUTECHANGE);    //RTC_C_CALENDAREVENT_HOURCHANGE
    }
    //Enable interrupt for RTC Ready Status, which asserts when the RTC
    //Calendar registers are ready to read.
    //Also, enable interrupts for the Calendar alarm and Calendar event.
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIE + RTCAIE);    //RTCRDYIE +
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIFG + RTCAIFG);    //RTCRDYIFG +
    //RTCPS0CTL &= ~(RT0PSIE|RTCIV_RT0PSIFG);//RTCIV_RT0PSIFG
    RTC_C_disableInterrupt(RTC_C_BASE,
    RTC_C_PRESCALE_TIMER0_INTERRUPT);
    //Start RTC Clock
    RTC_C_startClock(RTC_C_BASE);
}
void Init_RTC_C_N_gps(void)
{
    uint8_t i;
    uint8_t tmp;
    switch (Send_times_of_day)
    {
        case 0x00:
        times_0 = Send_time_reference;
        hoursAlarm_value = times_0;
        break;
        case 0x01:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);

        for(i=0;i<2;i++)
        {
            times_1[i] = ((i*12)+time_hour_temp);
            tmp = hourchange24(times_1[i]);
            times_1[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_1,2);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        if (time_hour_temp>=times_1[0])
        {
            time_temp = times_1[1];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        else
        {
            time_temp = times_1[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        break;
        case 0x02:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<4;i++)
        {
            times_2[i] = ((i*6)+time_hour_temp);
            tmp = hourchange24(times_2[i]);
            times_2[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_2,4);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<3;i++)
        {
            if ((time_hour_temp>=times_2[i]) && (time_hour_temp<=times_2[i+1]))
            {
                time_temp = times_2[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_2[3])||(time_hour_temp>=times_2[0]))
        {
            time_temp = times_2[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x03:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<6;i++)
        {
            times_3[i] = ((i*4)+time_hour_temp);
            tmp = hourchange24(times_3[i]);
            times_3[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_3,6);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<5;i++)
        {
            if ((time_hour_temp>=times_3[i]) && (time_hour_temp<=times_3[i+1]))
            {
                time_temp = times_3[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_3[5])||(time_hour_temp<times_3[0]))
        {
            time_temp = times_3[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x04:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<12;i++)
        {
            times_4[i] = ((i*2)+time_hour_temp);
            tmp = hourchange24(times_4[i]);
            times_4[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_4,12);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<11;i++)
        {
            if ((time_hour_temp>=times_4[i]) && (time_hour_temp<=times_4[i+1]))
            {
                time_temp = times_4[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_4[11])||(time_hour_temp<times_4[0]))
        {
            time_temp = times_4[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x05:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        __no_operation();                //for debug
        for(i=0;i<24;i++)
        {
            times_5[i] = i;
            //times_5[i] = hourchange24(times_5[i]);
        }
        tmp0 = time_Stamp[0];
        time_hour_temp = itoa_bcd_dec(tmp0);
        __no_operation();                //for debug
        for(i=0;i<23;i++)
        {
            if ((time_hour_temp>=times_5[i]) && (time_hour_temp<=times_5[i+1]))
            {
                time_temp = times_5[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if(time_hour_temp>=times_5[23])
        {
            time_temp = times_5[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        __no_operation();                //for debug
        break;
        default:
        Send_times_of_day = 0x00;
        hoursAlarm_value = Send_time_reference;
        //hoursAlarm_value = Send_time_reference;
        break;
    }
//    //minute_count = 0;
//    //Setup Current Time for Calendar
//    currentTime.Seconds = time_Stamp[2];
//    currentTime.Minutes = time_Stamp[1];
//    currentTime.Hours = time_Stamp[0];
//    currentTime.DayOfWeek = 0x03;
//    currentTime.DayOfMonth = 0x10;
//    currentTime.Month = 0x10;
//    currentTime.Year = 0x2018;

    //Initialize Calendar Mode of RTC
    /*
     * Base Address of the RTC_A
     * Pass in current time, intialized above
     * Use BCD as Calendar Register Format
     */
//    RTC_C_initCalendar(RTC_C_BASE, &currentTime,
//    RTC_C_FORMAT_BCD);
    //hoursAlarm_value=hourchangebcd_fifo(Send_time_reference,Send_times_of_day);
    //Setup Calendar Alarm for 5:00pm on the 5th day of the week.
    //Note: Does not specify day of the week.
    RTC_C_configureCalendarAlarmParam param = { 0 };
    param.minutesAlarm = 0x00;
    param.hoursAlarm = hoursAlarm_value;    //Send_time_reference;
    param.dayOfWeekAlarm = RTC_C_ALARMCONDITION_OFF;
    param.dayOfMonthAlarm = RTC_C_ALARMCONDITION_OFF;
    RTC_C_configureCalendarAlarm(RTC_C_BASE, &param);

//    //Specify an interrupt to assert every minute
    if (intergral_point_flag == 0x01)
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_HOURCHANGE);    //RTC_C_CALENDAREVENT_HOURCHANGE
    }
    else
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_MINUTECHANGE); //RTC_C_CALENDAREVENT_HOURCHANGEMINUTECHANGE
    }

    //Enable interrupt for RTC Ready Status, which asserts when the RTC
    //Calendar registers are ready to read.
    //Also, enable interrupts for the Calendar alarm and Calendar event.
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIE + RTCAIE);    //RTCRDYIE +
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIFG + RTCAIFG);    //RTCRDYIFG +
    //RTCPS0CTL &= ~(RT0PSIE|RTCIV_RT0PSIFG);//RTCIV_RT0PSIFG
    RTC_C_disableInterrupt(RTC_C_BASE,
    RTC_C_PRESCALE_TIMER0_INTERRUPT);
    //Start RTC Clock
    RTC_C_startClock(RTC_C_BASE);
}
void Init_RTC_C_gps(void)
{
    uint8_t i;
    uint8_t tmp;
    switch (Send_times_of_day)
    {
        case 0x00:
        times_0 = Send_time_reference;
        hoursAlarm_value = times_0;
        break;
        case 0x01:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);

        for(i=0;i<2;i++)
        {
            times_1[i] = ((i*12)+time_hour_temp);
            tmp = hourchange24(times_1[i]);
            times_1[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_1,2);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        if (time_hour_temp>=times_1[0])
        {
            time_temp = times_1[1];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        else
        {
            time_temp = times_1[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);

        }
        break;
        case 0x02:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<4;i++)
        {
            times_2[i] = ((i*6)+time_hour_temp);
            tmp = hourchange24(times_2[i]);
            times_2[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_2,4);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<3;i++)
        {
            if ((time_hour_temp>=times_2[i]) && (time_hour_temp<=times_2[i+1]))
            {
                time_temp = times_2[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_2[3])||(time_hour_temp>=times_2[0]))
        {
            time_temp = times_2[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x03:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<6;i++)
        {
            times_3[i] = ((i*4)+time_hour_temp);
            tmp = hourchange24(times_3[i]);
            times_3[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_3,6);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<5;i++)
        {
            if ((time_hour_temp>=times_3[i]) && (time_hour_temp<=times_3[i+1]))
            {
                time_temp = times_3[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_3[5])||(time_hour_temp<times_3[0]))
        {
            time_temp = times_3[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x04:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        for(i=0;i<12;i++)
        {
            times_4[i] = ((i*2)+time_hour_temp);
            tmp = hourchange24(times_4[i]);
            times_4[i] = tmp;
            tmp = 0;
            //times_4[i] = hourchange24(times_4[i]);
        }
        sort(times_4,12);
        time_hour_temp = itoa_bcd_dec(time_Stamp[0]);
        for(i=0;i<11;i++)
        {
            if ((time_hour_temp>=times_4[i]) && (time_hour_temp<=times_4[i+1]))
            {
                time_temp = times_4[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if((time_hour_temp>=times_4[11])||(time_hour_temp<times_4[0]))
        {
            time_temp = times_4[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        break;
        case 0x05:
        time_hour_temp = itoa_bcd_dec(Send_time_reference);
        __no_operation();                //for debug
        for(i=0;i<24;i++)
        {
            times_5[i] = i;
            //times_5[i] = hourchange24(times_5[i]);
        }
        tmp0 = time_Stamp[0];
        time_hour_temp = itoa_bcd_dec(tmp0);
        __no_operation();                //for debug
        for(i=0;i<23;i++)
        {
            if ((time_hour_temp>=times_5[i]) && (time_hour_temp<=times_5[i+1]))
            {
                time_temp = times_5[i+1];
                hoursAlarm_value = itoa_dec_bcd(time_temp);
            }

        }
        if(time_hour_temp>=times_5[23])
        {
            time_temp = times_5[0];
            hoursAlarm_value = itoa_dec_bcd(time_temp);
        }
        __no_operation();                //for debug
        break;
        default:
        Send_times_of_day = 0x00;
        hoursAlarm_value = Send_time_reference;
        break;
    }
    //minute_count = 0;
    //Setup Current Time for Calendar
    currentTime.Seconds = time_Stamp[2];
    currentTime.Minutes = time_Stamp[1];
    currentTime.Hours = time_Stamp[0];
    currentTime.DayOfWeek = 0x03;
    currentTime.DayOfMonth = 0x10;
    currentTime.Month = 0x10;
    currentTime.Year = 0x2018;

    //Initialize Calendar Mode of RTC
    /*
     * Base Address of the RTC_A
     * Pass in current time, intialized above
     * Use BCD as Calendar Register Format
     */
    RTC_C_initCalendar(RTC_C_BASE, &currentTime,
    RTC_C_FORMAT_BCD);
    //hoursAlarm_value=hourchangebcd_fifo(Send_time_reference,Send_times_of_day);
    //Setup Calendar Alarm for 5:00pm on the 5th day of the week.
    //Note: Does not specify day of the week.
    RTC_C_configureCalendarAlarmParam param = { 0 };
    param.minutesAlarm = 0x00;
    param.hoursAlarm = hoursAlarm_value;    //Send_time_reference;
    param.dayOfWeekAlarm = RTC_C_ALARMCONDITION_OFF;
    param.dayOfMonthAlarm = RTC_C_ALARMCONDITION_OFF;
    RTC_C_configureCalendarAlarm(RTC_C_BASE, &param);

//    //Specify an interrupt to assert every minute
    if (intergral_point_flag == 0x01)
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_HOURCHANGE);    //RTC_C_CALENDAREVENT_HOURCHANGE
    }
    else
    {
        RTC_C_setCalendarEvent(RTC_C_BASE,
        RTC_C_CALENDAREVENT_MINUTECHANGE); //RTC_C_CALENDAREVENT_HOURCHANGEMINUTECHANGE
    }

    //Enable interrupt for RTC Ready Status, which asserts when the RTC
    //Calendar registers are ready to read.
    //Also, enable interrupts for the Calendar alarm and Calendar event.
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIE + RTCAIE);    //RTCRDYIE +
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIFG + RTCAIFG);    //RTCRDYIFG +
    //RTCPS0CTL &= ~(RT0PSIE|RTCIV_RT0PSIFG);//RTCIV_RT0PSIFG
    RTC_C_disableInterrupt(RTC_C_BASE,
    RTC_C_PRESCALE_TIMER0_INTERRUPT);
    //Start RTC Clock
    RTC_C_startClock(RTC_C_BASE);
}
void Init_RTC_C(void)
{
    minute_count = 0;
    //Setup Current Time for Calendar
    currentTime.Seconds = time_Stamp[2];
    currentTime.Minutes = time_Stamp[1];
    currentTime.Hours = time_Stamp[0];
    currentTime.DayOfWeek = 0x03;
    currentTime.DayOfMonth = 0x10;
    currentTime.Month = 0x10;
    currentTime.Year = 0x2018;

    //Initialize Calendar Mode of RTC
    /*
     * Base Address of the RTC_A
     * Pass in current time, intialized above
     * Use BCD as Calendar Register Format
     */
    RTC_C_initCalendar(RTC_C_BASE, &currentTime,
    RTC_C_FORMAT_BCD);

    //Setup Calendar Alarm for 5:00pm on the 5th day of the week.
    //Note: Does not specify day of the week.
    RTC_C_configureCalendarAlarmParam param = { 0 };
    param.minutesAlarm = 0x00;
    param.hoursAlarm = hoursAlarm_value;    //Send_time_reference;
    param.dayOfWeekAlarm = RTC_C_ALARMCONDITION_OFF;
    param.dayOfMonthAlarm = RTC_C_ALARMCONDITION_OFF;
    RTC_C_configureCalendarAlarm(RTC_C_BASE, &param);

    //Specify an interrupt to assert every minute
    RTC_C_setCalendarEvent(RTC_C_BASE,
    RTC_C_CALENDAREVENT_MINUTECHANGE);    //RTC_C_CALENDAREVENT_HOURCHANGE

    //Enable interrupt for RTC Ready Status, which asserts when the RTC
    //Calendar registers are ready to read.
    //Also, enable interrupts for the Calendar alarm and Calendar event.
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIE + RTCAIE);    //RTCRDYIE +
    RTC_C_enableInterrupt(RTC_C_BASE,
    RTCTEVIFG + RTCAIFG);    //RTCRDYIFG +
    //RTCPS0CTL &= ~(RT0PSIE|RTCIV_RT0PSIFG);//RTCIV_RT0PSIFG
    RTC_C_disableInterrupt(RTC_C_BASE,
    RTC_C_PRESCALE_TIMER0_INTERRUPT);
    //Start RTC Clock
    RTC_C_startClock(RTC_C_BASE);
}
/*
 * sensor and gps data is framed
 */
void all_data_fram(void)
{
    int16_t i;
    sensors_data[47] = 0x00;
    sensors_data[0] = 0xEE;
    sensors_data[1] = 0x90;
    for (i = 0; i < 47; i++)
        sensors_data[47] ^= sensors_data[i];
#if send_sms_data
    for (i = 0; i < 48; i++)
        sms_data[i] = sensors_data[i];
//    HexToDsp(sms_data,str_sms,48);
    Sdk_Str2BcdStr(sms_data, 48, sms_data_bk);
#endif
    framLog_fifo(sensors_data);
    for (i = 19; i < 35; i++)
        sensors_data[i] = 0x00;

}
/*
 * enter LPM3 mode
 */
void ultral_low_power()
{
    __bis_SR_register(LPM3_bits + GIE);
}
/*
 * reset timer0
 */
void reset_timer0()
{
    Timer_A_stop(TIMER_A0_BASE); //disable timerA0
    timer0_count_1 = 0;
    tmp1 = 0;
    Init_Clock();
    Timer_A_clear(TIMER_A0_BASE);
    Init_Timer0_A0();
}
/*
 * reset timer0
 */
void reset_timer4()
{
    Timer_A_stop(TIMER_A4_BASE); //disable timerA4
    //timer4_gps_flag = 0;
    timer4_count_0 = 0;
    tmp0 = 0;
    Timer_A_clear(TIMER_A4_BASE);
    Init_Timer4_A0();            //if gps data is valid,reset timer4
}
/*
 * sensor data processing,include hdc1080 and bmp280,battery voltage
 */
void sensor_data_process()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5); //open sensor power
    __delay_cycles(40000);
    I2CInitialise(); //I2C start
    HDC1000_Init();
    temperatureData = HDC1000_TempRead(); //HDC1080
    //            StartLuxMeasurement();
    //            __delay_cycles(1000000);
    //            luxData = GetLuxValue(); //OPT3001
    bmp280_Init_0();
    g_u32ActualPress = read_all();
    g_u32ActualPress = g_u32ActualPress / 256;
    if (Temperature_T > 0)
    {
        Temperature_T_temp = Temperature_T;
        Temperature_T_temp = (Temperature_T_temp | 0x8000);
    }
    else
    {
        Temperature_T_temp = abs(Temperature_T);
    }
    sensors_data[21] = ((Temperature_T_temp & 0xff00) >> 8);  //temperature
    sensors_data[22] = (Temperature_T_temp & 0x00ff);
    sensors_data[23] = (Humidity_T >> 8);
    sensors_data[24] = Humidity_T;
    sensors_data[31] = ((g_u32ActualPress & 0xff000000) >> 24);  //actualpress
    sensors_data[32] = ((g_u32ActualPress & 0xff0000) >> 16);
    sensors_data[33] = ((g_u32ActualPress & 0xff00) >> 8);
    sensors_data[34] = g_u32ActualPress;
    //            lux = (int) (uint16_t) luxData;
    //            sensors_data[25] = (lux >> 8);  //humidity
    //            sensors_data[26] = lux;
    //Altitude_T = (int) ((760 - ((g_u32ActualPress + 42) * 3 / 4)) * 12);
//    Altitude_T = (uint16_t) ((double) ((760
//            - ((g_u32ActualPress / 25600) * 3 / 4)) * 12));
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5); //close sensor power
    //ADC12_B_disable(ADC12_B_BASE);
    //ADC12_B_enable(ADC12_B_BASE);
    //ADC12_B_clearInterrupt(ADC12_B_BASE, 0,
    //ADC12_B_IFG0);
    adc12_b_initialise();  //votage
    adc12b_start_conversion();
    voltage_T = ((voltage / 4096) * 1.2) * 6 * 1000;
    sensors_data[29] = (voltage_T >> 8);
    sensors_data[30] = voltage_T;
    if (voltage_T < voltage_threshold_tmp)
    {
        alarm_data[29] = (voltage_T >> 8);
        alarm_data[30] = voltage_T;
        send_frame_alarm = 0x01;
        alarm_data[3] = 0x01;
    }
    else
    {
        send_frame_alarm = 0x00;
        alarm_data[3] = 0x00;
        alarm_data[2] = 0x00;
    }
}

/*
 * time data processing
 */
uint8_t hourchangebcd(uint8_t time, uint8_t time_offset)
{
    time_temp = 0;
    time_flag = 0;
    time_temp = time + time_offset;
    __no_operation();  //for debug
    switch (time_temp)
    {
    case 0:
        time_flag = 0x00;
        break;
    case 1:
        time_flag = 0x01;
        break;
    case 2:
        time_flag = 0x02;
        break;
    case 3:
        time_flag = 0x03;
        break;
    case 4:
        time_flag = 0x04;
        break;
    case 5:
        time_flag = 0x05;
        break;
    case 6:
        time_flag = 0x06;
        break;
    case 7:
        time_flag = 0x07;
        break;
    case 8:
        time_flag = 0x08;
        break;
    case 9:
        time_flag = 0x09;
        break;
    case 10:
        time_flag = 0x10;
        break;
    case 11:
        time_flag = 0x11;
        break;
    case 12:
        time_flag = 0x12;
        break;
    case 13:
        time_flag = 0x13;
        break;
    case 14:
        time_flag = 0x14;
        break;
    case 15:
        time_flag = 0x15;
        break;
    case 16:
        time_flag = 0x16;
        break;
    case 17:
        time_flag = 0x17;
        break;
    case 18:
        time_flag = 0x18;
        break;
    case 19:
        time_flag = 0x19;
        break;
    case 20:
        time_flag = 0x20;
        break;
    case 21:
        time_flag = 0x21;
        break;
    case 22:
        time_flag = 0x22;
        break;
    case 23:
        time_flag = 0x23;
        break;
    case 24:
        time_flag = 0x00;
        break;
    case 25:
        time_flag = 0x01;
        break;
    case 26:
        time_flag = 0x02;
        break;
    case 27:
        time_flag = 0x03;
        break;
    case 28:
        time_flag = 0x04;
        break;
    case 29:
        time_flag = 0x05;
        break;
    case 30:
        time_flag = 0x06;
        break;
    case 31:
        time_flag = 0x07;
        break;
    case 32:
        time_flag = 0x08;
        break;
    case 33:
        time_flag = 0x09;
        break;
    case 34:
        time_flag = 0x10;
        break;
    case 35:
        time_flag = 0x11;
        break;
    case 36:
        time_flag = 0x12;
        break;
    case 37:
        time_flag = 0x13;
        break;
    case 38:
        time_flag = 0x14;
        break;
    case 39:
        time_flag = 0x15;
        break;
    case 40:
        time_flag = 0x16;
        break;
    case 41:
        time_flag = 0x17;
        break;
    case 42:
        time_flag = 0x18;
        break;
    case 43:
        time_flag = 0x19;
        break;
    case 44:
        time_flag = 0x20;
        break;
    case 45:
        time_flag = 0x21;
        break;
    case 46:
        time_flag = 0x22;
        break;
    case 47:
        time_flag = 0x23;
        break;
    case 48:
        time_flag = 0x00;
        break;
    default:
        time_flag = 0x09;
        break;
    }
    //time_flag = 0;
    return time_flag;
}
/*
 * time data processing
 */
uint8_t hourchange24(uint8_t time)
{
    time_flag = 0;
    __no_operation();  //for debug
    switch (time)
    {
    case 0:
        time_flag = 0x00;
        break;
    case 1:
        time_flag = 0x01;
        break;
    case 2:
        time_flag = 0x02;
        break;
    case 3:
        time_flag = 0x03;
        break;
    case 4:
        time_flag = 0x04;
        break;
    case 5:
        time_flag = 0x05;
        break;
    case 6:
        time_flag = 0x06;
        break;
    case 7:
        time_flag = 0x07;
        break;
    case 8:
        time_flag = 0x08;
        break;
    case 9:
        time_flag = 0x09;
        break;
    case 10:
        time_flag = 10;
        break;
    case 11:
        time_flag = 11;
        break;
    case 12:
        time_flag = 12;
        break;
    case 13:
        time_flag = 13;
        break;
    case 14:
        time_flag = 14;
        break;
    case 15:
        time_flag = 15;
        break;
    case 16:
        time_flag = 16;
        break;
    case 17:
        time_flag = 17;
        break;
    case 18:
        time_flag = 18;
        break;
    case 19:
        time_flag = 19;
        break;
    case 20:
        time_flag = 20;
        break;
    case 21:
        time_flag = 21;
        break;
    case 22:
        time_flag = 22;
        break;
    case 23:
        time_flag = 23;
        break;
    case 24:
        time_flag = 0;
        break;
    case 25:
        time_flag = 1;
        break;
    case 26:
        time_flag = 2;
        break;
    case 27:
        time_flag = 3;
        break;
    case 28:
        time_flag = 4;
        break;
    case 29:
        time_flag = 5;
        break;
    case 30:
        time_flag = 6;
        break;
    case 31:
        time_flag = 7;
        break;
    case 32:
        time_flag = 8;
        break;
    case 33:
        time_flag = 9;
        break;
    case 34:
        time_flag = 10;
        break;
    case 35:
        time_flag = 11;
        break;
    case 36:
        time_flag = 12;
        break;
    case 37:
        time_flag = 13;
        break;
    case 38:
        time_flag = 14;
        break;
    case 39:
        time_flag = 15;
        break;
    case 40:
        time_flag = 16;
        break;
    case 41:
        time_flag = 17;
        break;
    case 42:
        time_flag = 18;
        break;
    case 43:
        time_flag = 19;
        break;
    case 44:
        time_flag = 20;
        break;
    case 45:
        time_flag = 21;
        break;
    case 46:
        time_flag = 22;
        break;
    case 47:
        time_flag = 23;
        break;
    case 48:
        time_flag = 0;
        break;
    default:
        break;
    }
    //time_flag = 0;
    return time_flag;
}
void sort(uint8_t* a, uint8_t max)
{
    uint8_t i, j;
    int temp;
    for (i = 0; i < max; i++)
    {
        for (j = i + 1; j < max; j++)
        {
            if (a[j] < a[i])
            {
                temp = a[j];
                a[j] = a[i];
                a[i] = temp;
            }
        }
    }
}
/*
 * gprs data processing
 */
void gprs_data_process(void)
{
    //Timer_A_stop(TIMER_A4_BASE);//disable timerA4
    Init_UART_A1();
    __delay_cycles(4000);
    //framLog(alarm_data);
    //framLog_fifo(sensors_data);
    //sendDataFRAM();
    sendDataFRAM_fifo();
#if send_sms_data
    __delay_cycles(4000);
    UART_A1_Printf("+++");    //exit from transparent transfer mode
    __delay_cycles(8000000);
    sendDataSMS();
#endif
    EUSCI_A_UART_disableInterrupt(EUSCI_A1_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT); // Disable interrupt
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1); //close sim power
    data_ok_flag = 0;
    send_frame_alarm = 0;
    //gprs_send_ok = 1;
    //gps_time[0] = 0;
    //timer4_count_0 = 0;
    //tmp0 = 0;
    //gprs_flag = 0;
    //Timer_A_clear(TIMER_A4_BASE);
    //Init_Timer4_A0();
}
void gprs_data_process_alarm(void)
{

    //Timer_A_stop(TIMER_A4_BASE);//disable timerA4
    Init_UART_A1();
    __delay_cycles(4000);
    framLog(alarm_data);
    sendDataFRAM();
    //sendDataFRAM_fifo();
    EUSCI_A_UART_disableInterrupt(EUSCI_A1_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT); // Disable interrupt
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1); //close sim power
    data_ok_flag = 0;
    send_frame_normal = 0;
    send_frame_alarm = 0;
    alarm_data[2] = 0x00;    //复位标志
    alarm_data[3] = 0x00;    //低压标志
}
/*
 * gps data processing
 */
void gps_data_process()
{
    //uint8_t i;
    Init_UART_A0();  //gps start
    __delay_cycles(8000);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);  //open gps power
    __delay_cycles(8000000);
    UART_A0_Printf("$PQTXT,W,0,1*23\r\n");
    UART_A0_Printf("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); //SET_NMEA_OUTPUT-GMC GGA
//    UART_A0_Printf("$PMTK869,1,1*35\r\n");  //Set easy mode
    Timer_A_clear(TIMER_A1_BASE);
    Init_Timer1_A0();
    while (1)
    {
        if ((gps_easy_flag == 1) && (rec_ok == 0))
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); //close gps power
            gps_easy_flag = 0;
            newTime = RTC_C_getCalendarTime(RTC_C_BASE);
            time_Stamp[2] = newTime.Seconds;
            time_Stamp[1] = newTime.Minutes;
            time_Stamp[0] = newTime.Hours;
//            RTC_C_holdClock(RTC_C_BASE);
            Init_RTC_C_N_gps();
//            sensors_data[8] |= 0x10;      //经度H,未定位状态指示
            break;
        }
        if (rec_ok)
        {
//            gps_location_count++;
//            if (gps_location_count == 0x01)
//            {
//                gps_first_location = 0x01;
//            }
//            else
//            {
//                gps_location_count = 0x02;
//                gps_first_location = 0x00;
//            }
            Timer_A_stop(TIMER_A1_BASE);  //disable timerA1
            Timer_A_clear(TIMER_A1_BASE);
            timer1_count_0 = 0;
//            gps_location_flag = 1;
            gps_easy_flag = 0;
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); //close gps power
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); //close gps power
            sensors_data[1] = 0x90;  //上电复位标志
            EUSCI_A_UART_disableInterrupt(
            EUSCI_A0_BASE,
                                          EUSCI_A_UART_RECEIVE_INTERRUPT); // Disable interrupt
            sensors_data[2] = WD_a; //纬度方向
            sensors_data[3] = ((WD[0] - 0x30) << 4) + (WD[1] - 0x30); //纬度
            sensors_data[4] = ((WD[2] - 0x30) << 4) + (WD[3] - 0x30); //纬分H
            sensors_data[5] = ((WD[5] - 0x30) << 4) + (WD[6] - 0x30); //纬分M
            sensors_data[6] = ((WD[7] - 0x30) << 4) + (WD[8] - 0x30); //纬分L
            sensors_data[7] = JD_a;  //经度方向
            if (Gps_Data_Valid == 'A')
                sensors_data[8] = JD[0] - 0x30;      //经度H
            else
                sensors_data[8] = JD[0];
            sensors_data[9] = ((JD[1] - 0x30) << 4) + (JD[2] - 0x30); //经度L
            sensors_data[10] = ((JD[3] - 0x30) << 4) + (JD[4] - 0x30); //经分H
            sensors_data[11] = ((JD[6] - 0x30) << 4) + (JD[7] - 0x30); //经分M
            sensors_data[12] = ((JD[8] - 0x30) << 4) + (JD[9] - 0x30); //经分L
            sscanf(altitude, "%d", &Altitude_T);
            if (Altitude_T > 0)
            {
                Altitude_T_temp = Altitude_T;
                Altitude_T_temp = (Altitude_T_temp | 0x8000);
            }
            else
            {
                Altitude_T_temp = abs(Altitude_T);
            }
            sensors_data[27] = ((Altitude_T_temp & 0xff00) >> 8);  //Altitude
            sensors_data[28] = (Altitude_T_temp & 0x00ff);
            sensors_data[41] = ((utc[8] - 0x30) << 4)  //time
            + (utc[9] - 0x30);
            sensors_data[40] = ((utc[5] - 0x30) << 4) + (utc[7] - 0x30);
            sensors_data[39] = ((utc[3] - 0x30) << 4) + (utc[4] - 0x30);
            sensors_data[38] = ((utc[1] - 0x30) << 4) + (utc[2] - 0x30);
            sensors_data[37] = (utc[0] - 0x30);
            sensors_data[44] = ((date[4] - 0x30) << 4)  //date
            + (date[5] - 0x30);
            sensors_data[43] = ((date[2] - 0x30) << 4) + (date[3] - 0x30);
            sensors_data[42] = ((date[0] - 0x30) << 4) + (date[1] - 0x30);
            Gps_Data_Valid = 0;
            rec_ok = 0;
            gps_time_convert();
            //time_hour_temp = time_Stamp[0];
//            if (gps_first_location == 0x01)
//            {
//                RTC_C_holdClock(RTC_C_BASE);
//                Init_RTC_C_gps_first();
//            }
//            else
//            {
//            RTC_C_holdClock(RTC_C_BASE);
            Init_RTC_C_gps();
//            }
            break;
        }
    }
//    RTC_C_holdClock(RTC_C_BASE);
//    Init_RTC_C();//reset rtc with local time
}
uint8_t itoa_dec_bcd(uint8_t tmp)
{
    uint8_t temp = 0, temp_flag = 0;
    temp_flag = tmp;
    switch (temp_flag)
    {
    case 0:
        temp = 0x00;
        break;
    case 1:
        temp = 0x01;
        break;
    case 2:
        temp = 0x02;
        break;
    case 3:
        temp = 0x03;
        break;
    case 4:
        temp = 0x04;
        break;
    case 5:
        temp = 0x05;
        break;
    case 6:
        temp = 0x06;
        break;
    case 7:
        temp = 0x07;
        break;
    case 8:
        temp = 0x08;
        break;
    case 9:
        temp = 0x09;
        break;
    case 10:
        temp = 0x10;
        break;
    case 11:
        temp = 0x11;
        break;
    case 12:
        temp = 0x12;
        break;
    case 13:
        temp = 0x13;
        break;
    case 14:
        temp = 0x14;
        break;
    case 15:
        temp = 0x15;
        break;
    case 16:
        temp = 0x16;
        break;
    case 17:
        temp = 0x17;
        break;
    case 18:
        temp = 0x18;
        break;
    case 19:
        temp = 0x19;
        break;
    case 20:
        temp = 0x20;
        break;
    case 21:
        temp = 0x21;
        break;
    case 22:
        temp = 0x22;
        break;
    case 23:
        temp = 0x23;
        break;
    case 24:
        temp = 0x00;
        break;
    case 25:
        temp = 0x01;
        break;
    case 26:
        temp = 0x02;
        break;
    case 27:
        temp = 0x03;
        break;
    case 28:
        temp = 0x04;
        break;
    case 29:
        temp = 0x05;
        break;
    case 30:
        temp = 0x06;
        break;
    case 31:
        temp = 0x07;
        break;
    case 32:
        temp = 0x08;
        break;
    default:
        temp = 0x00;
        break;
    }
    return temp;
}
uint8_t itoa_bcd_dec(uint8_t tmp)
{
    uint8_t temp, temp_flag;
    temp_flag = tmp;
    switch (temp_flag)
    {
    case 0x00:
        temp = 0;
        break;
    case 0x01:
        temp = 1;
        break;
    case 0x02:
        temp = 2;
        break;
    case 0x03:
        temp = 3;
        break;
    case 0x04:
        temp = 4;
        break;
    case 0x05:
        temp = 5;
        break;
    case 0x06:
        temp = 6;
        break;
    case 0x07:
        temp = 7;
        break;
    case 0x08:
        temp = 8;
        break;
    case 0x09:
        temp = 9;
        break;
    case 0x10:
        temp = 10;
        break;
    case 0x11:
        temp = 11;
        break;
    case 0x12:
        temp = 12;
        break;
    case 0x13:
        temp = 13;
        break;
    case 0x14:
        temp = 14;
        break;
    case 0x15:
        temp = 15;
        break;
    case 0x16:
        temp = 16;
        break;
    case 0x17:
        temp = 17;
        break;
    case 0x18:
        temp = 18;
        break;
    case 0x19:
        temp = 19;
        break;
    case 0x20:
        temp = 20;
        break;
    case 0x21:
        temp = 21;
        break;
    case 0x22:
        temp = 22;
        break;
    case 0x23:
        temp = 23;
        break;
    case 0x24:
        temp = 0;
        break;
    default:
        break;
    }
    return temp;
}
/*
 *
 * GPS time conversion
 */
void gps_time_convert()
{
//    uint8_t i, temp, time_flag_count = 0;
//    gps_time[0] = (((utc[0] - 0x30) * 10) + (utc[1] - 0x30)) + 8; //beijing time hours
//    gps_time[1] = ((utc[2] - 0x30) * 10) + (utc[3] - 0x30); //minutes
//    gps_time[2] = ((utc[4] - 0x30) * 10) + (utc[5] - 0x30); //seconds
    time_Stamp[0] = (((utc[0] - 0x30) * 10)  //时
    + (utc[1] - 0x30) + 8);
    time_Stamp[1] = ((utc[2] - 0x30) << 4)  //分
    + (utc[3] - 0x30);
    time_Stamp[2] = ((utc[4] - 0x30) << 4)  //秒
    + (utc[5] - 0x30);
//    temp = time_Stamp[0];
//    for (i = 0; i < 24; i++)
//    {
//        if (temp != time_gps_data[i])
//            time_flag_count++;
//    }
//    if (time_flag_count == 24)
//    {
//        time_Stamp[0] = 0x00;
//        time_flag_count = 0x00;
//    }
    time_Stamp[0] = itoa_dec_bcd(time_Stamp[0]);
//    time_Stamp[1] = itoa_dec_bcd(time_Stamp[1]);
//    time_Stamp[2] = itoa_dec_bcd(time_Stamp[2]);

}
/*
 * Enter Low Power Mode 3.5
 */
void enterLPM35()
{
    // Configure button S2 (P5.5) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN5,
    GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN5);

    // Request the disabling of the core voltage regulator when device enters
    // LPM3 (or LPM4) so that we can effectively enter LPM3.5 (or LPM4.5).
    PMM_turnOffRegulator();

    //Enter LPM3 mode with interrupts enabled
    __bis_SR_register(LPM4_bits + GIE);
    __no_operation();    //for debug
}

//*****************************************************************************
// Support functions
//*****************************************************************************
void Init_Timer0_A0(void)
{
    // Start timer
    Timer_A_initUpModeParam param = { 0 };
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 0x8000;    //1 second
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &param);
}

//*****************************************************************************
// Support functions
//*****************************************************************************
void Init_Timer4_A0(void)
{
    // Start timer
    Timer_A_initUpModeParam param = { 0 };
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 0x4000;    //500ms second
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A4_BASE, &param);
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768, 0);
    //Set ACLK=LFXT
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //Start XT1 with no time out
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
}

/*
 * USCI_A0 Interrupt Service Routine that receives PC GUI's commands
 */
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    int tmp;
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        tmp = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
        switch (tmp)
        {
        case '$':
            cmd_number = 0;        //命令类型清空
            mode = 1;                //接收命令模式
            byte_count = 0;        //接收位数清空
            break;
        case ',':
            seg_count++;        //逗号计数加1
            byte_count = 0;
            break;

        default:
            if (mode == 1)
            {
                //命令种类判断
                cmd[byte_count] = tmp;            //接收字符放入类型缓存
                if (byte_count >= 4)
                {                //如果类型数据接收完毕，判断类型
                    if (cmd[0] == 'G')
                    {
                        if (cmd[1] == 'P')
                        {
                            if (cmd[2] == 'G')
                            {
                                if (cmd[3] == 'G')
                                {
                                    if (cmd[4] == 'A')
                                    {
                                        cmd_number = 1;
                                        mode = 2;
                                        seg_count = 0;
                                        byte_count = 0;
                                    }
                                }
                            }
                            else if (cmd[2] == 'R')
                            {
                                if (cmd[3] == 'M')
                                {
                                    if (cmd[4] == 'C')
                                    {
                                        cmd_number = 2;
                                        mode = 2;
                                        seg_count = 0;
                                        byte_count = 0;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else if (mode == 2)
            {
                //接收数据处理
                switch (cmd_number)
                {
                case 1:                //类型1数据接收。GPGGA
                    switch (seg_count)
                    {
                    case 7:                                //定位使用的卫星数
                        if (byte_count < 2)
                        {
                            Number_of_SV[byte_count] = tmp;
                        }
                        break;
                    case 8:                                //精度因子
                        if (byte_count < 5)
                        {
                            hdop[byte_count] = tmp;
                        }
                        switch (byte_count)
                        {
                        case 3:
                            sensors_data[20] = ((hdop[2] - 0x30) << 4)
                                    + (hdop[3] - 0x30);
                            sensors_data[19] = (hdop[0] - 0x30);
                            rec_ok_T++;
                            break;
                        case 4:
                            sensors_data[20] = ((hdop[3] - 0x30) << 4)
                                    + (hdop[4] - 0x30);
                            sensors_data[19] = ((hdop[0] - 0x30) << 4)
                                    + (hdop[1] - 0x30);
                            rec_ok_T++;
                            break;
                        default:
                            break;
                        }
                        if ((Gps_Data_Valid == 'A') && (rec_ok_T == 0x04))
                        {
                            rec_ok_T = 0;
                            rec_ok = 1;
                        }
                        break;
                    case 9:                                //高度处理

                        if (byte_count < 8)
                        {
                            altitude[byte_count] = tmp;
                        }
                        break;
                    default:
                        break;
                    }

                    break;
                case 2:                //类型2数据接收。GPRMC
                    switch (seg_count)
                    {
                    case 1:
                        if (byte_count < 10)
                        {                //UTC Time
                            utc[byte_count] = tmp;
                        }
//                        switch (byte_count)
//                        {
//                        case 9:
//                            sensors_data[41] = ((utc[8] - 0x30) << 4)
//                                    + (utc[9] - 0x30);
//                            sensors_data[40] = ((utc[5] - 0x30) << 4)
//                                    + (utc[7] - 0x30);
//                            sensors_data[39] = ((utc[3] - 0x30) << 4)
//                                    + (utc[4] - 0x30);
//                            sensors_data[38] = ((utc[1] - 0x30) << 4)
//                                    + (utc[2] - 0x30);
//                            sensors_data[37] = (utc[0] - 0x30);
//                            break;
//                        default:
//                            break;
//                        }
                        break;
                    case 2:
                        if (byte_count < 1)
                        {                //定位信息处理 A=有效定位，V=无效定位
                            Gps_Data_Valid = tmp;
                        }
                        break;
                    case 3:                                //纬度处理ddmm.mmmm
                        if (byte_count < 9)
                        {
                            WD[byte_count] = tmp;
                        }
                        break;
                    case 4:                                //纬度方向处理N
                        if (byte_count < 1)
                        {
                            WD_a = tmp;
                        }
                        break;
                    case 5:                                //经度处理dddmm.mmmm
                        if (byte_count < 10)
                        {
                            JD[byte_count] = tmp;
                        }
                        break;
                    case 6:                                //经度方向处理E
                        if (byte_count < 1)
                        {
                            JD_a = tmp;
                        }
                        break;
                    case 7:                                //速度处理000.00~999.99 节
                        speed[byte_count] = tmp;
                        switch (byte_count)
                        {
                        case 3:
                            sensors_data[15] = ((speed[2] - 0x30) << 4)
                                    + (speed[3] - 0x30);
                            sensors_data[14] = (speed[0] - 0x30);
                            sensors_data[13] = 0x00;
                            break;
                        case 4:
                            sensors_data[15] = ((speed[3] - 0x30) << 4)
                                    + (speed[4] - 0x30);
                            sensors_data[14] = ((speed[0] - 0x30) << 4)
                                    + (speed[1] - 0x30);
                            sensors_data[13] = 0x00;
                            break;
                        case 5:
                            sensors_data[15] = ((speed[4] - 0x30) << 4)
                                    + (speed[5] - 0x30);
                            sensors_data[14] = ((speed[1] - 0x30) << 4)
                                    + (speed[2] - 0x30);
                            sensors_data[13] = (speed[0] - 0x30);
                            ;
                            break;
                        default:
                            break;
                        }
                        break;
                    case 8:                                //航向处理000.00~359.99 度
                        course[byte_count] = tmp;
                        switch (byte_count)
                        {
                        case 3:
                            sensors_data[18] = ((course[2] - 0x30) << 4)
                                    + (course[3] - 0x30);
                            sensors_data[17] = (course[0] - 0x30);
                            sensors_data[16] = 0x00;
                            break;
                        case 4:
                            sensors_data[18] = ((course[3] - 0x30) << 4)
                                    + (course[4] - 0x30);
                            sensors_data[17] = ((course[0] - 0x30) << 4)
                                    + (course[1] - 0x30);
                            sensors_data[16] = 0x00;
                            break;
                        case 5:
                            sensors_data[18] = ((course[4] - 0x30) << 4)
                                    + (course[5] - 0x30);
                            sensors_data[17] = ((course[1] - 0x30) << 4)
                                    + (course[2] - 0x30);
                            sensors_data[16] = (course[0] - 0x30);

                            break;
                        default:
                            break;
                        }
                        break;
                    case 9:
                        if (byte_count < 6)
                        {                //DATE
                            date[byte_count] = tmp;
                        }
//                        switch (byte_count)
//                        {
//                        case 5:
//                            sensors_data[44] = ((date[4] - 0x30) << 4)
//                                    + (date[5] - 0x30);
//                            sensors_data[43] = ((date[2] - 0x30) << 4)
//                                    + (date[3] - 0x30);
//                            sensors_data[42] = ((date[0] - 0x30) << 4)
//                                    + (date[1] - 0x30);
//                            break;
//                        default:
//                            break;
//                        }
                        //if(Gps_Data_Valid=='A') rec_ok = 1;
                    default:
                        break;
                    }
                    break;
                default:
                    break;
                }
            }
            byte_count++;        //接收数位加1
            break;
        }

        //            __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
        break;
    case USCI_UART_UCTXIFG:
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        break;
    }
}
/*
 * USCI_A1 Interrupt Service Routine that receives SIM800C's commands
 */
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    //    int tmp;
    switch (__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        //tmp = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);
        Rec_Server_Data();
        //            __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
        break;
    case USCI_UART_UCTXIFG:
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        break;
    }
}
/*
 * Timer0_A0 Interrupt Vector (TAIV) handler
 *
 *
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    //uint8_t tmp1 = 0;
    timer0_count_1++;
    switch (Collect_time_interval)
    {
        case 0x06:
        if (timer0_count_1 == (Timer_Count_H-Timer_data_collector))
        {
            Timer_A_stop(TIMER_A0_BASE);        //disable timerA0
            tmp1 = 0;
            timer0_count_1 = 0;
            timer_flag_h = 1;
            __bic_SR_register_on_exit(LPM3_bits);// Exit active CPU
        }
        break;
        case 0x0C:
        if (timer0_count_1 == (Timer_Count_2M-Timer_data_collector))
        {
            Timer_A_stop(TIMER_A0_BASE);        //disable timerA0
            tmp1 = 0;
            timer0_count_1 = 0;
            timer_flag_h = 1;
            __bic_SR_register_on_exit(LPM3_bits);// Exit active CPU
        }
        break;
        case 0x12:
        if (timer0_count_1 == (Timer_Count_3H-Timer_data_collector))
        {
            Timer_A_stop(TIMER_A0_BASE);        //disable timerA0
            tmp1 = 0;
            timer0_count_1 = 0;
            timer_flag_h = 1;
            __bic_SR_register_on_exit(LPM3_bits);// Exit active CPU
        }
        break;
        default:
        if (timer0_count_1 == (Timer_Count_10M-Timer_data_collector)) //reserved 2 hours
        {
            Timer_A_stop(TIMER_A0_BASE);        //disable timerA0
            tmp1 = 0;
            timer0_count_1 = 0;
            timer_flag_h = 1;
            __bic_SR_register_on_exit(LPM3_bits);// Exit active CPU
        }
        break;
    }
}
/*
 * Timer1_A0 Interrupt Vector (TAIV) handler
 *
 *
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
    timer1_count_0++;
    if (timer1_count_0 == gps_over_time)
    {
        Timer_A_stop(TIMER_A1_BASE);    //disable timerA1
        Timer_A_clear(TIMER_A1_BASE);
        timer1_count_0 = 0;
        gps_easy_flag = 1;
    }

//__bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
}
/*
 * Timer3_A0 Interrupt Vector (TAIV) handler
 *
 *
 */
#pragma vector=TIMER3_A0_VECTOR
__interrupt void TIMER3_A0_ISR(void)
{
    timer3_count_0++;
    if (timer3_count_0 == Timer_Count_M)
    {
        Timer_A_stop(TIMER_A3_BASE);    //disable timerA1
        timer3_count_0 = 0;
        gprs_flag = 1;
    }
//__bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
}
/*
 * Timer2_A0 Interrupt Vector (TAIV) handler
 *
 *
 */
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
{
    if (Timer2_start)
        Times++;
    if (Times == shijian)
    {
        Timer2_start = 0;
        Times = 0;
    }

//__bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
}
/*
 * Timer4_A0 Interrupt Vector (TAIV) handler
 *
 */
#pragma vector=TIMER4_A0_VECTOR
__interrupt void TIMER4_A0_ISR(void)
{
    WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL;
}
/*
 * ADC12 Interrupt Service Routine
 *
 */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    switch (__even_in_range(ADC12IV, 76))
    {
    case ADC12IV_NONE:
        break;                // Vector  0:  No interrupt
    case ADC12IV_ADC12OVIFG:
        break;          // Vector  2:  ADC12MEMx Overflow
    case ADC12IV_ADC12TOVIFG:
        break;         // Vector  4:  Conversion time overflow
    case ADC12IV_ADC12HIIFG:
        break;          // Vector  6:  ADC12HI
    case ADC12IV_ADC12LOIFG:
        break;          // Vector  8:  ADC12LO
    case ADC12IV_ADC12INIFG:
        break;           // Vector 10:  ADC12IN
    case ADC12IV_ADC12IFG0:                   // Vector 12:  ADC12MEM0
        ADC12IFGR0 &= ~ADC12IFG0;             // Clear interrupt flag
        //__bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
        break;
    case ADC12IV_ADC12IFG1:                   // Vector 14:  ADC12MEM1
        ADC12IFGR0 &= ~ADC12IFG1;             // Clear interrupt flag
        //__bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
        break;
    case ADC12IV_ADC12IFG2:
        break;            // Vector 16:  ADC12MEM2
    case ADC12IV_ADC12IFG3:
        break;            // Vector 18:  ADC12MEM3
    case ADC12IV_ADC12IFG4:
        break;            // Vector 20:  ADC12MEM4
    case ADC12IV_ADC12IFG5:
        break;            // Vector 22:  ADC12MEM5
    case ADC12IV_ADC12IFG6:
        break;            // Vector 24:  ADC12MEM6
    case ADC12IV_ADC12IFG7:
        break;            // Vector 26:  ADC12MEM7
    case ADC12IV_ADC12IFG8:
        break;            // Vector 28:  ADC12MEM8
    case ADC12IV_ADC12IFG9:
        break;            // Vector 30:  ADC12MEM9
    case ADC12IV_ADC12IFG10:
        break;           // Vector 32:  ADC12MEM10
    case ADC12IV_ADC12IFG11:
        break;           // Vector 34:  ADC12MEM11
    case ADC12IV_ADC12IFG12:
        break;           // Vector 36:  ADC12MEM12
    case ADC12IV_ADC12IFG13:
        break;           // Vector 38:  ADC12MEM13
    case ADC12IV_ADC12IFG14:
        break;           // Vector 40:  ADC12MEM14
    case ADC12IV_ADC12IFG15:
        ADC12IFGR0 &= ~ADC12IFG15;             // Clear interrupt flag
        break;           // Vector 42:  ADC12MEM15

    case ADC12IV_ADC12IFG16:
        break;           // Vector 44:  ADC12MEM16
    case ADC12IV_ADC12IFG17:
        break;           // Vector 46:  ADC12MEM17
    case ADC12IV_ADC12IFG18:
        break;           // Vector 48:  ADC12MEM18
    case ADC12IV_ADC12IFG19:
        break;           // Vector 50:  ADC12MEM19
    case ADC12IV_ADC12IFG20:
        break;           // Vector 52:  ADC12MEM20
    case ADC12IV_ADC12IFG21:
        break;           // Vector 54:  ADC12MEM21
    case ADC12IV_ADC12IFG22:
        break;           // Vector 56:  ADC12MEM22
    case ADC12IV_ADC12IFG23:
        break;           // Vector 58:  ADC12MEM23
    case ADC12IV_ADC12IFG24:
        break;           // Vector 60:  ADC12MEM24
    case ADC12IV_ADC12IFG25:
        break;           // Vector 62:  ADC12MEM25
    case ADC12IV_ADC12IFG26:
        break;           // Vector 64:  ADC12MEM26
    case ADC12IV_ADC12IFG27:
        break;           // Vector 66:  ADC12MEM27
    case ADC12IV_ADC12IFG28:
        break;           // Vector 68:  ADC12MEM28
    case ADC12IV_ADC12IFG29:
        break;           // Vector 70:  ADC12MEM29
    case ADC12IV_ADC12IFG30:
        break;           // Vector 72:  ADC12MEM30
    case ADC12IV_ADC12IFG31:
        break;           // Vector 74:  ADC12MEM31
    case ADC12IV_ADC12RDYIFG:
        break;          // Vector 76:  ADC12RDY
    default:
        break;
    }
}
/*
 * Port 3 interrupt service routine to TLE4913
 *
 */
#pragma vector=PORT3_VECTOR
__interrupt void Port_3(void)
{
    if (P3IFG & BIT7)          //get button push interrupt
    {
        if ((P3IN & 0x80) == 0x00)
        {
            tle4913_flag = 1;
            P3IFG &= ~BIT7;     //clear P2.0 interrupt flag
            //P2IE &= ~BIT0;
            P3IES ^= BIT7;    //convert the trigger conditions
            __bic_SR_register_on_exit(LPM3_bits);     // Exit LPM4
            __bic_SR_register_on_exit(LPM4_bits);     // Exit LPM4
            RTC_C_startClock(RTC_C_BASE);
        }
        else if ((P3IN & 0x80) == 0x80)
        {
            tle4913_flag = 0;
            P3IFG &= ~BIT7;     //clear P2.0 interrupt flag
            //P2IE &= ~BIT0;
            P3IES ^= BIT7;    //convert the trigger conditions
//            Init_Timer4_A0();
//            Init_Timer0_A0();
            WDTCTL = WDTPW | 0xff00;
        }
//        P3IFG &= ~BIT7;     //clear P2.0 interrupt flag
//        P3IES ^= BIT7;    //convert the trigger conditions
    }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_ISR(void)
{
    switch (__even_in_range(RTCIV, 16))
    {
    case RTCIV_NONE:
        __no_operation();
        break;  //No interrupts
    case RTCIV_RTCOFIFG:
        __no_operation();
        break;  //RTCOFIFG
    case RTCIV_RTCRDYIFG:         //RTCRDYIFG
        //Toggle P1.0 every second
//                GPIO_toggleOutputOnPin(
//                        GPIO_PORT_P1,
//                        GPIO_PIN0);
        //newTime = RTC_C_getCalendarTime(RTC_C_BASE);
        __no_operation();
        break;
    case RTCIV_RTCTEVIFG:         //RTCEVIFG
        //Interrupts every minute
        __no_operation();
        if (intergral_point_flag == 0x01)
        {
            if (rtc_times > 0)
                rtc_times--;
            if (rtc_times == 0)
            {
                timer_flag_h = 1;
                rtc_times = rtc_times_count;
                __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
            }
        }
        else
        {
//            if (gps_first_location == 0x01)
//            {
//                timer_flag_h = 1;
//                minute_count++;
//                __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
//
//            }
//            else
//            {
            minute_count++;
            //newTime = RTC_C_getCalendarTime(RTC_C_BASE);
            if (minute_count >= 10)
            {
                minute_count = 0;
                if (rtc_times > 0)
                    rtc_times--;
                if (rtc_times == 0)
                {
                    timer_flag_h = 1;
                    rtc_times = Collect_time_interval;
                    //rtc_times = 0x01;
                    __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
                }
            }
//            }
        }
        //Read out New Time a Minute Later BREAKPOINT HERE
//        newTime = RTC_C_getCalendarTime(RTC_C_BASE);

        break;
    case RTCIV_RTCAIFG:         //RTCAIFG
        //Interrupts 5:00pm on 5th day of week
        data_ok_flag = 1;
        //rtc_alarm_count++;
        __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
        //__no_operation();
        break;
    case RTCIV_RT0PSIFG:
        __no_operation();
        break; //RT0PSIFG
    case RTCIV_RT1PSIFG:
        __no_operation();
        break; //RT1PSIFG
    default:
        __no_operation();
        break;
    }
}


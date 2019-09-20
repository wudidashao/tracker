/*
 * config.h
 *
 *  Created on: 2018年7月16日
 *      Author: zlzhang
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#define send_sms_data    1
#define UUID_H      0x93
#define UUID_L      0x0F
#define gps_over_time Timer_Count_3M
//for timer
#define Timer_Count_S               1
#define Timer_Count_30S             30
#define Timer_Count_M               60
#define Timer_Count_2M              120
#define Timer_Count_3M              180
#define Timer_Count_5M              300
#define Timer_Count_10M             600
#define Timer_Count_30M             1800
#define Timer_Count_H               3600
#define Timer_Count_2H              7200
#define Timer_Count_3H              10800
#define COMPARE_VALUE               4096
#pragma PERSISTENT(reset_alarm_count)
uint8_t reset_alarm_count= 0;
uint8_t intergral_point_flag = 0,rtc_times_count = 0;
uint8_t rtc_times = 0x01, minute_count = 0, rtc_alarm_count = 1;
uint8_t send_times_count=8 ,time_hour_temp = 0;
uint8_t hoursAlarm_value = 0x18,gps_location_count=0,gps_first_location=0;
uint8_t time_temp=0,time_flag=0,send_frame_normal=0,send_frame_alarm = 0;
uint8_t times_0,times_1[2],times_2[4],times_3[6],times_4[12],times_5[24];
uint16_t voltage_threshold_tmp = 3700;
uint16_t Send_time_offset_tmp = 0;
Calendar currentTime;
volatile Calendar newTime;
//***** Global Data *****
// votage
int16_t voltage_T;
//HDC1080
int16_t Temperature_T;
uint16_t Temperature_T_temp;
int16_t Humidity_T;
// OPT3001
uint16_t rawData;
uint16_t lux;
float convertedLux;
// BME280
s32 returnRslt = 0;
s32 g_s32ActualTemp = 0;
uint32_t g_u32ActualPress = 0; //300-1100hPa ,+9000m--500m
u32 g_u32ActualHumity = 0;
int Altitude_T;
uint16_t Altitude_T_temp;
//Sensor Status Variables
int32_t luxData;            //stored Lux value
int32_t temperatureData;      //Variable to store temperature data from HDC1000
int32_t humidityData;         //Variable to store humidity data from HDC1000
//GPS数据存储数组
uint8_t utc_temp0, utc_temp1, date_temp0, date_temp1, utc_default[11],
        date_default[6];
uint16_t temp_altitude;
uint8_t gps_time[3] = { 0, 30, 30 };   //H,M,S,default am 9:00 send data by gprs
uint8_t utc[11]; //TIME IN FORMAT 'HHMMSS.SSS'
uint8_t date[6]; //DATA IN FORMAT 'DDMMHH'
char utc_char[] = "003030.000"; //TIME IN FORMAT 'HHMMSS.SSS'
char date_char[] = "021018"; //DATA IN FORMAT 'DDMMHH'
uint8_t JD[10];           //lONGGITUDE IN FORMAT 'DDDMM.MMMMM'
uint8_t JD_a;               //‘E’=East ‘W’=West
uint8_t WD[9];             //lATITUDE IN FORMAT 'DDMM.MMMMM'
uint8_t WD_a;             //‘N’=North ‘S’=South
uint8_t speed[6];          //SPEED
char altitude[8] = "-123.8";         //高度
uint8_t course[6];        //航向
uint8_t hdop[5];          //水平精度因子
uint8_t Number_of_SV[2];        //使用的卫星数
uint8_t Gps_Data_Valid;           //定位状态
uint8_t sensors_data[] = {0xEE, 0x90, 0x4E, 0x31, 0x16, 0x45, 0x87, 0x45, 0x01,
                           0x20, 0x39, 0x91, 0x48, 0x00, 0x00, 0x61, 0x01, 0x41,
                           0x78, 0x02, 0x54, 0xFE, 0x70, 0x00, 0x00, 0x08, 0xC1,
                           0xFE, 0xA4, 0x10, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x42, 0x03, 0x02, 0x17, 0x03, 0x07, 0x18,
                           UUID_H,
                           UUID_L, 0x00 };       //EE 90
uint8_t alarm_data[] = {0xEE, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           UUID_H,
                           UUID_L, 0x00 };       //EE 90

uint8_t time_gps_data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                           0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                           0x18, 0x19, 0x20, 0x21, 0x22, 0x23};
uint8_t rec_ok = 0, rec_ok_T = 0, gprs_send_ok = 0, gps_send_ok = 0,
        data_ok_flag = 0;
uint8_t tle4913_flag = 0;
uint8_t collector_times = 2, rtc_time = 9;
uint8_t gps_easy_flag = 0, gprs_flag = 0, gps_location_flag = 0;
//串口中断需要的变量
uint8_t seg_count;        //逗号计数器
uint8_t dot_count;        //小数点计数器
uint8_t byte_count;        //位数计数器
uint8_t cmd_number;        //命令类型
uint8_t mode;                //0：结束模式，1：命令模式，2：数据模式
uint8_t cmd[5];            //命令类型存储数组
//for timer
uint8_t tmp1 = 0, tmp0 = 0;
uint16_t Timer_data_collector = Timer_Count_M;
uint16_t Timer_data_collector_interval = Timer_Count_2H;
//uint16_t Timer_data_updata = Timer_Count_H;
uint16_t timer0_count_1, timer4_count_0;
extern uint16_t timer1_count_0, timer3_count_0, timer2_count_0;
extern uint8_t Times, shijian;
volatile unsigned char Timer2_start;   //定时器2延时启动计数器
uint8_t timer_flag_h, timer_gps_h, timer4_gps_flag = 0; //flag for sampling interval
extern float voltage;

//***** Function Prototypes *****
void Init_Clock(void);
void enterLPM35();
void Init_Timer0_A0(void);
void Init_Timer4_A0(void);
void gps_time_convert(void);
void ultral_low_power(void);
void reset_timer0(void);
void reset_timer4(void);
void sensor_data_process(void);
void gps_data_process(void);
void gprs_data_process(void);
void all_data_fram(void);;
uint8_t itoa_bcd_dec(uint8_t tmp);
uint8_t itoa_dec_bcd(uint8_t tmp);
uint8_t hourchange24(uint8_t time);
void Init_RTC_C(void);
void Init_RTC_C_gps(void);
void Init_RTC_C_N_gps(void);
void Init_RTC_C_gps_first(void);
void Init_RTC_C_reset_gprs(void);
void gprs_data_process_alarm(void);
void sort (uint8_t* a,uint8_t max);
uint8_t hourchangebcd(uint8_t time,uint8_t time_offset);
#endif /* CONFIG_H_ */

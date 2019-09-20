/*
 * USCA_UART.h
 *
 *  Created on: 2017Äê12ÔÂ13ÈÕ
 *      Author: vladimir
 */

#ifndef USCA_UART_H_
#define USCA_UART_H_

#endif /* USCA_UART_H_ */

extern uint8_t config_data[];
#define Tle4913_threshold          config_data[0]
#define Send_time_reference        config_data[1]
#define Send_time_offset           config_data[2]
#define Collect_time_interval      config_data[3]
#define Positioning_time_max       config_data[4]
#define Gprs_send_time_threshold   config_data[5]
#define voltage_threshold          config_data[6]
#define Gprs_time_over             config_data[7]
#define Send_times_of_day          config_data[8]
//#if config_data_flag
//#if defined(__TI_COMPILER_VERSION__)
//#pragma DATA_SECTION(config_data,".infoA")
//#elif defined(__IAR_SYSTEMS_ICC__)
//#pragma location="INFOA"
//__no_init
//#endif
void sendDataSMS(void);
void command_analysis(void);
void Init_UART_A0(void);
void Init_UART_A1(void);
void Init_UART_A3(void);
void UART_A0_Printf(uint8_t *ptr);
void UART_A1_Printf(uint8_t *ptr);
void UART_A3_Printf(uint8_t *ptr);
void UART_A3_SendnBytes(unsigned char *ptr,unsigned char n);
void CLR_Buf2(void);
uint8_t Find(char *a);
void Second_AT_Command(char *b,char *a,uint8_t wait_time);
void Second_AT_Command_tcp(char *b,char *a,uint8_t wait_time);
void Wait_CREG(void);
void Set_ATE0(void);
void Connect_Server(void);
void Rec_Server_Data(void);
void Init_Gprs(void);
void Init_Timer1_A0(void);
void Init_Timer2_A0(void);
void Init_Timer3_A0(void);
int8_t HexToAsc(int8_t aChar);
int8_t AscToHex(int8_t hex);
void HexArrayToString(unsigned char *hexarray,uint8_t length,unsigned char *string);
void HexToDsp(uint8_t *hex, uint8_t *dsp, uint8_t hex_count);
void Sdk_Int2Char(uint8_t p_nNum, uint8_t *p_Ch);
void Sdk_Str2BcdStr(uint8_t *p_Str, uint8_t p_nLen, uint8_t *p_StrBcd);

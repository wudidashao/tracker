/*
 * USCA_UART.c
 *
 *  Created on: 2017��12��13��
 *      Author: vladimir
 */

/*
 * UART Communication Initialization
 */

#include "driverlib.h"
#include "string.h"
#include "FRAMLogMode.h"
#include "usca_uart.h"
#pragma PERSISTENT(config_data)
uint8_t config_data[20]=
{
 0x00,//config_data[0]:Tle4913_threshold;
 0x18,//config_data[1]:Send_time_reference;
 0x00,//config_data[2]:Send_time_offset;
 0x0c,//config_data[3]:Collect_time_interval;
 0x0c,//config_data[4]:Positioning_time_threshold;
 0x06,//config_data[5]:Gprs_send_time_threshold;
 0x19,//config_data[6]:Voltage_threshold;
 0x00,//config_data[7]:Send_time_threshold;
 0x00 //config_data[8]:Send_times_of_day;
};
#define Buf2_Max 200    //UCA1 buffer length
/*************  Local constant declaration  **************/
//const char *string = "AT+CIPSTART=\"TCP\",\"112.33.20.107\",8966"; //IP��¼������,��Ҫ�����Լ���IP�Ͷ˿����޸�
const char *string_dns = "AT+CIPSTART=\"TCP\",\"www.birdsdata.com\",8966"; //IP��¼������,��Ҫ�����Լ���IP�Ͷ˿����޸�
const char *string = "AT+CIPSTART=\"TCP\",\"114.215.18.87\",8966"; //IP��¼������,��Ҫ�����Լ���IP�Ͷ˿����޸�
const unsigned char Num2CharTable[] = "0123456789ABCDEF";
/*************  ���ر�������  **************/

char Uart2_Buf[Buf2_Max]; //����2���ջ���
char *p1, *p2;
uint8_t Times = 0, shijian = 0, First_Int = 0;
bool gprs_ok = 0, AT_flag = 0;
bool dns_flag = 1;
extern uint8_t rtc_times_count;
extern uint8_t intergral_point_flag;
extern uint8_t collector_times, rtc_time;
extern uint8_t gprs_flag;
extern uint8_t gps_easy_flag;
extern uint8_t command_ok_flag;
extern uint8_t rtc_times;
extern uint8_t send_times_count;
extern uint16_t voltage_threshold_tmp;
extern uint16_t Send_time_offset_tmp;
extern int16_t count_capture,count_tx;
extern uint8_t sms_data_bk[];
extern uint8_t sms_data[];
extern uint8_t *str_sms;
uint16_t timer1_count_0, timer3_count_0, timer2_count_0;
volatile unsigned char Timer2_start;   //��ʱ��2��ʱ����������
//void UART_A1_Printf(uint8_t *ptr);
//void UART_A3_Printf(uint8_t *ptr);
//void Wait_CREG(void);
//void Connect_Server(void);
//void Init_UART_A0(void);

void Init_Timer1_A0(void)
{
    // Start timer
    Timer_A_initUpModeParam param = { 0 };
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 0x8000;   //1 second
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A1_BASE, &param);

    __bis_SR_register(GIE);
    //For debugger
    __no_operation();
    // Change timer delay to 1/8 second
    //Timer_A_setCompareValue(TIMER_A0_BASE,
    //                        TIMER_A_CAPTURECOMPARE_REGISTER_0,
    //                       0x1000
    //                       );
    //Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
}
//*****************************************************************************
// Initial Timer2_A0
//*****************************************************************************
void Init_Timer2_A0(void)
{
    // Start timer
    Timer_A_initUpModeParam param = { 0 };
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 0x8000;    //0.125 second
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A2_BASE, &param);

    __bis_SR_register(GIE);
    //For debugger
    //__no_operation();
    // Change timer delay to 1/8 second
    //Timer_A_setCompareValue(TIMER_A0_BASE,
    //                        TIMER_A_CAPTURECOMPARE_REGISTER_0,
    //                       0x1000
    //                       );
    //Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
}
//*****************************************************************************
// Support functions
//*****************************************************************************
void Init_Timer3_A0(void)
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
    Timer_A_initUpMode(TIMER_A3_BASE, &param);
}
void Init_UART_A0()
{
    // Configure P2.0 - UCA0TXD and P2.1 - UCA0RXD
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN1,
                                               GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0,
                                                GPIO_SECONDARY_MODULE_FUNCTION);
    // Configure UART
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 0x34;
    param.firstModReg = 1;
    param.secondModReg = 0x49;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
        return;

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT);
    // Enable USCI_A3 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

    // Enable globale interrupt
    //__bis_SR_register(GIE);
    __enable_interrupt();
}
void Init_UART_A1()
{
    // Configure P2.5 - UCA0TXD and P2.6 - UCA0RXD
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN6,
                                               GPIO_SECONDARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,
                                                GPIO_SECONDARY_MODULE_FUNCTION);
    // Configure UART
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 4;
    param.firstModReg = 5;
    param.secondModReg = 0x55;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A1_BASE, &param))
        return;

    EUSCI_A_UART_enable(EUSCI_A1_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT);
    // Enable USCI_A3 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

    // Enable globale interrupt
    //__bis_SR_register(GIE);
    __enable_interrupt();
}
void Init_UART_A3()
{
    // Configure UART
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 4;
    param.firstModReg = 5;
    param.secondModReg = 0x55;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A3_BASE, &param))
        return;

    EUSCI_A_UART_enable(EUSCI_A3_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A3_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A3 RX interrupt
    //EUSCI_A_UART_enableInterrupt(EUSCI_A3_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

    // Enable globale interrupt
    //__bis_SR_register(GIE);
    __enable_interrupt();
}

void UART_A0_Printf(uint8_t *ptr)
{
    while (*ptr != '\0')
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *ptr++);         // send string
    }
    while (EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY))
        ;
}
void UART_A1_Printf(uint8_t *ptr)
{
    while (*ptr != '\0')
    {
        EUSCI_A_UART_transmitData(EUSCI_A1_BASE, *ptr++);         // send string
    }
    //    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A1_BASE, EUSCI_A_UART_BUSY));
}
void UART_A3_Printf(uint8_t *ptr)
{
    while (*ptr != '\0')
    {
        EUSCI_A_UART_transmitData(EUSCI_A3_BASE, *ptr++);         // send string
    }
    //    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A3_BASE, EUSCI_A_UART_BUSY));
}

void UART_A3_SendnBytes(uint8_t *ptr, uint8_t n)                        //����n���ֽ�
{
    while ((n--) != 0)
    {
        EUSCI_A_UART_transmitData(EUSCI_A3_BASE, *ptr);
        ptr++;
    }
    //    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A3_BASE, EUSCI_A_UART_BUSY));
}
/*******************************************************************************
 * ������ : CLR_Buf2
 * ����   : �������2��������
 * ����   :
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void CLR_Buf2(void)
{
    uint16_t k;
    for (k = 0; k < Buf2_Max; k++)      //��������������
    {
        Uart2_Buf[k] = 0x00;
    }
    p1 = Uart2_Buf;               //�����ַ�������ʼ�洢λ��
}
/*******************************************************************************
 * ������ : Find
 * ����   : �жϻ������Ƿ���ָ�����ַ���
 * ����   :
 * ���   :
 * ����   : unsigned char:1 �ҵ�ָ���ַ���0 δ�ҵ�ָ���ַ�
 * ע��   :
 *******************************************************************************/
uint8_t Find(char *a)
{
    if (strstr(Uart2_Buf, a) != NULL)
        return 1;
    else
        return 0;
}
/*******************************************************************************
 * ������ : Second_AT_Command_tcp
 * ����   : ����ATָ���
 * ����   : �������ݵ�ָ�롢���͵ȴ�ʱ��(��λ��S)
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void Second_AT_Command_tcp(char *b, char *a, uint8_t wait_time)
{

    AT_flag = 0;
    First_Int = 0;
    char *c;
    c = b;                                      //�����ַ�����ַ��c
    CLR_Buf2();
    b = c;                          //���ַ�����ַ��b
    for (; *b != '\0'; b++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A1_BASE, *b);
    }
    UART_A1_Printf("\r\n");
    __delay_cycles(40000000);
    if (Find(a))
    {
        AT_flag = 0;
    }
    else
    {
        AT_flag = 1;
    }
//    Times = 0;
//    shijian = wait_time;
//    Timer2_start = 1;
//    Timer_A_clear(TIMER_A2_BASE);
//    Init_Timer2_A0();

//    while(i == 0)
//    {
//        if(!Find(a))
//        {
//            i = 1;
//
//            if(Timer2_start == 0)
//            {
//                b = c;                          //���ַ�����ַ��b
//                for (; *b!='\0';b++)
//                {
//                    //while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
//                    //USART_SendData(USART2,*b);//UART2_SendData(*b);
//                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE,*b);
//                }
//                UART2_SendLR();
//                UART_A1_Printf("\r\n");
//                Times = 0;
//                shijian = wait_time;
//                Timer2_start = 1;
//                Timer_A_clear(TIMER_A2_BASE);
//                Init_Timer2_A0();
//            }
//        }
//        else
//        {
//            i = 1;
//            //AT_flag = 1;
//            Timer2_start = 0;
//            Timer_A_stop(TIMER_A2_BASE);//disable timerA2
//        }
//    }
    CLR_Buf2();
    First_Int = 0;

}
/*******************************************************************************
 * ������ : Second_AT_Command
 * ����   : ����ATָ���
 * ����   : �������ݵ�ָ�롢���͵ȴ�ʱ��(��λ��S)
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void Second_AT_Command(char *b, char *a, uint8_t wait_time)
{
    AT_flag = 0;
    First_Int = 0;
    char *c;
    c = b;                                      //�����ַ�����ַ��c
    CLR_Buf2();
    b = c;                          //���ַ�����ַ��b
    for (; *b != '\0'; b++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A1_BASE, *b);
    }
    UART_A1_Printf("\r\n");
    __delay_cycles(8000000);
    if (Find(a))
    {
        AT_flag = 1;
    }
    else
    {
        AT_flag = 0;
    }
//    Times = 0;
//    shijian = wait_time;
//    Timer2_start = 1;
//    Timer_A_clear(TIMER_A2_BASE);
//    Init_Timer2_A0();

//    while(i == 0)
//    {
//        if(!Find(a))
//        {
//            i = 1;
//
//            if(Timer2_start == 0)
//            {
//                b = c;                          //���ַ�����ַ��b
//                for (; *b!='\0';b++)
//                {
//                    //while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
//                    //USART_SendData(USART2,*b);//UART2_SendData(*b);
//                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE,*b);
//                }
//                UART2_SendLR();
//                UART_A1_Printf("\r\n");
//                Times = 0;
//                shijian = wait_time;
//                Timer2_start = 1;
//                Timer_A_clear(TIMER_A2_BASE);
//                Init_Timer2_A0();
//            }
//        }
//        else
//        {
//            i = 1;
//            //AT_flag = 1;
//            Timer2_start = 0;
//            Timer_A_stop(TIMER_A2_BASE);//disable timerA2
//        }
//    }
    CLR_Buf2();
    First_Int = 0;
}
/*******************************************************************************
 * ������ : Wait_CREG
 * ����   : �ȴ�ģ��ע��ɹ�
 * ����   :
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void Wait_CREG(void)
{
    uint8_t i;
    uint8_t k;
    First_Int = 0;
    i = 0;
    Init_Timer3_A0();
    while (i == 0)
    {
        CLR_Buf2();
        UART_A1_Printf("AT+CREG?\r\n");   //��ѯ�ȴ�ģ��ע��ɹ�
        //Delay_nMs(5000);
        //__delay_cycles(40000000);
        //CLR_Buf2();
        //UART_A1_Printf("AT+CCID\r\n");   //��ѯ�ȴ�ģ��ע��ɹ�
        //Delay_nMs(5000);
        __delay_cycles(40000000);
//        while(1)//test
//        {
//        UART_A1_Printf("AT+CREG?\r\n");   //��ѯ�ȴ�ģ��ע��ɹ�
//        __delay_cycles(8000000);
//        UART_A1_Printf("AT+CSQ\r\n");   //��ѯ�ź�����
//        __delay_cycles(40000000);
//        }
        //UART_A1_Printf("AT+CPIN?\r\n");
        //UART_A3_Printf("ע����.....");
        if (gprs_flag)
        {
            Timer_A_stop(TIMER_A3_BASE);        //disable timerA1
            timer3_count_0 = 0;
            gprs_flag = 0;
            gprs_ok = 0;
            CLR_Buf2();
            break;
        }
        for (k = 0; k < Buf2_Max; k++)
        {
            if (Uart2_Buf[k] == ':')
            {
                if ((Uart2_Buf[k + 4] == '1') || (Uart2_Buf[k + 4] == '5')) //˵��ע��ɹ�
                {
                    i = 1;
                    //UART1_SendLR();
                    //UART_A3_Printf("\r\n");
                    //UART_A3_Printf("GPRSģ��ע��ɹ�\r\n");
                    timer3_count_0 = 0;
                    Timer_A_stop(TIMER_A3_BASE);               //disable timerA1
                    gprs_flag = 0;
                    CLR_Buf2();
                    gprs_ok = 1;
                    break;
                }
            }
        }

    }
}
/*******************************************************************************
 * ������ : Set_ATE0
 * ����   : ȡ������
 * ����   :
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void Set_ATE0(void)
{
    Second_AT_Command("ATE0", "OK", 3);                               //ȡ������
}
/*******************************************************************************
 * ������ : Connect_Server
 * ����   : GPRS���ӷ���������
 * ����   :
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void Connect_Server(void)
{
    //int i;
    //__disable_interrupt();
    //uint16_t i;
    //UART2_SendString("AT+CIPCLOSE=1");  //�ر�����
    //UART_A1_Printf("AT+CIPCLOSE=1");
    //UART_A1_Printf("\r\n");
    //Delay_nMs(100);

//Second_AT_Command("AT+CIPSHUT","SHUT OK",2);        //�ر��ƶ�����
//Second_AT_Command("AT+CGCLASS=\"B\"","OK",2);//����GPRS�ƶ�̨���ΪB,֧�ְ����������ݽ���
//Second_AT_Command("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",2);//����PDP������,��������Э��,��������Ϣ
    Second_AT_Command("AT+CGATT=1", "OK", 2);    //����GPRSҵ��
    if (AT_flag)
    {
        Second_AT_Command("AT+CIPMODE=1", "OK", 2);
        if (AT_flag)
        {
            Second_AT_Command("AT+CSTT=\"CMIOT\"", "OK", 2);    //����ΪGPRS����ģʽ
            if (AT_flag)
            {
                Second_AT_Command("AT+CIICR", "OK", 2);
                __delay_cycles(16000000);
                Second_AT_Command("AT+CIFSR", "", 2);
                if(dns_flag)
                {
                Second_AT_Command_tcp((char*) string_dns, "FAIL", 5);   //��������
                }
                else
                {
                    Second_AT_Command_tcp((char*) string, "FAIL", 5);   //��������
                }
            }
        }
    }
//if(AT_flag) Second_AT_Command("AT+CIPCSGP=1,\"CMIOT\"","OK",2);//����ΪGPRS����ģʽ
//if(AT_flag) Second_AT_Command("AT+CIPMUX=0","OK",2);//����Ϊ��·����
//if(AT_flag) Second_AT_Command("AT+CIPHEAD=1","OK",2);//���ý���������ʾIPͷ(�����ж�������Դ,���ڵ�·������Ч)
//if(AT_flag) Second_AT_Command("AT+CIPMODE=1","OK",2);//��͸������
//if(AT_flag) Second_AT_Command("AT+CSTT=\"CMIOT\"","OK",2);//����ΪGPRS����ģʽ
//if(AT_flag) Second_AT_Command("AT+CIICR","OK",2);
//if(AT_flag) Second_AT_Command("AT+CIFSR","",2);
////if(AT_flag) Second_AT_Command("AT+CIPCCFG=4,5,200,1","OK",2);//����͸��ģʽ�������ط�����:2,���1S����һ��,ÿ�η���200���ֽ�
//if(AT_flag) Second_AT_Command_tcp((char*)string,"CONNECT OK",5);//��������
//while(1);
    //Second_AT_Command("AT+CGATT=1","OK",2);
    //Second_AT_Command("AT+CIPMODE=1","OK",2);
    //Second_AT_Command("AT+CIPCSGP=1,\"CMNET\"","OK",2);//����ΪGPRS����ģʽ
    //Second_AT_Command("AT+CIICR","OK",2);
    /*    UART_A1_Printf("AT+CGATT=1\r\n");
     __delay_cycles(8000000);
     UART_A1_Printf("AT+CIPMODE=1\r\n");
     __delay_cycles(8000000);
     UART_A1_Printf("AT+CSTT=\"CMIOT\"\r\n");
     __delay_cycles(8000000);
     UART_A1_Printf("AT+CIICR\r\n");
     __delay_cycles(16000000);
     UART_A1_Printf("AT+CIFSR\r\n");
     __delay_cycles(16000000);
     UART_A1_Printf("AT+CIPSTART=\"TCP\",\"114.215.18.87\",8966\r\n");*/
    //UART_A1_Printf("AT+CIPSTART=\"TCP\",\"61.177.24.118\",9988\r\n");
    //UART_A1_Printf("AT+CIPSTART=\"TCP\",\"221.224.145.106\",9988\r\n");
    //__delay_cycles(16000000);

    CLR_Buf2();                                    //�崮�ڽ��ջ���Ϊ͸��ģʽ׼��
    //__enable_interrupt();

}
/*******************************************************************************
 * ������ : command_analysis
 * ����   : ����ָ�����
 * ����   :
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void command_analysis(void)
{
    //uint8_t i;
    uint8_t k = 0;
    int16_t j = 0;
    //First_Int = 0;
    for (k = 0; k < Buf2_Max; k++)
    {
        if ((Uart2_Buf[k] == 0xEA) && (Uart2_Buf[k + 1] == 0x90))
        {
            if(Uart2_Buf[k + 14]==0x00)
            {
                command_ok_flag = 1;

            }
            else if(Uart2_Buf[k + 14]==0xEE)
            {
                command_ok_flag = 0;
                if(dns_flag)
                {
                    dns_flag=0;
                }
                else
                {
                    dns_flag=1;
                }
            }
            //command_ok_flag = 1;
            Send_time_reference      = Uart2_Buf[k + 2];
            Send_time_offset         = Uart2_Buf[k + 3];
            Collect_time_interval    = Uart2_Buf[k + 4];
            Positioning_time_max     = Uart2_Buf[k + 5];
            Gprs_send_time_threshold = Uart2_Buf[k + 6];
            voltage_threshold        = Uart2_Buf[k + 7];
            Gprs_time_over           = Uart2_Buf[k + 8];
            Send_times_of_day        = Uart2_Buf[k + 9];
//            Send_times_of_day = 0x05;
//            Collect_time_interval =0x01;
            voltage_threshold_tmp = (3410 + voltage_threshold*10);
            Send_time_offset_tmp = (Send_time_offset*5*60);
        }
    }
    j = (count_capture-count_tx);
    if(j>165) Collect_time_interval = 0x18;
    if(Collect_time_interval==0x06)
    {
        intergral_point_flag = 0x01;
        rtc_times_count = 0x01;
    }
    else if(Collect_time_interval==0x0C)
    {
        intergral_point_flag = 0x01;
        rtc_times_count = 0x02;
    }
    else if(Collect_time_interval==0x12)
    {
        intergral_point_flag = 0x01;
        rtc_times_count = 0x03;
    }
    else if(Collect_time_interval==0x18)
    {
        intergral_point_flag = 0x01;
        rtc_times_count = 0x04;
    }
    else
    {
        intergral_point_flag = 0x00;
    }
    CLR_Buf2();
}
/*******************************************************************************
 * ������ : Rec_Server_Data
 * ����   : ���շ��������ݺ���,��ԭ������
 * ����   :
 * ���   :
 * ����   :
 * ע��   :
 *******************************************************************************/
void Rec_Server_Data(void)
{
    /*
     if(p2!=p1)          //˵����������δ����
     {
     //while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
     //USART_SendData(USART2,*p2);
     EUSCI_A_UART_transmitData(EUSCI_A1_BASE,*p2);
     p2++;
     if(p2>&Uart2_Buf[Buf2_Max])
     p2=Uart2_Buf;
     }
     */
    Uart2_Buf[First_Int] = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);
    ;     //�����յ����ַ����浽������
    First_Int++;                   //����ָ������ƶ�
    if (First_Int > Buf2_Max)         //���������,������ָ��ָ�򻺴���׵�ַ
    {
        First_Int = 0;
    }
}
void Init_Gprs()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);         //open sim power
    __delay_cycles(8000000);
    //UART_A3_Printf("GPRSģ��GPRS���Գ���\r\n");
    //UART_A3_Printf("GPRSģ����ע������\r\n");
    //UART_A1_Printf("+++");//�˳�͸��ģʽ������ģ�黹����͸��ģʽ��
    //__delay_cycles(16000000);
    Wait_CREG();   //�ȴ�ģ��ע��ɹ�
    //UART_A3_Printf("GPRSģ�鿪ʼ���ӷ�����\r\n");
    //Set_ATE0();    //ȡ������

}
void sendDataSMS(void)
{
    uint16_t i;
    Second_AT_Command("AT+CMGF=1", "OK", 2);
    Second_AT_Command("AT+CSCA=\"+8613800100569\"", "OK", 5);
    Second_AT_Command("AT+CMGS=\"1064899150267\"", ">", 5);
    if(AT_flag)
    {
//        for(i=0;i<96;i++)
//                {
//                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, str_sms[i]);
//                }
//    UART_A1_Printf(str_sms);
    for(i=0;i<96;i++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A1_BASE, sms_data_bk[i]);
    }
    __delay_cycles(40000);
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, 0x1A);
    UART_A1_Printf("\r\n");
    AT_flag = 0;
    __delay_cycles(40000000);
    }
}
//��������AscToHex()
//������������ASCIIת��Ϊ16����
int8_t AscToHex(int8_t hex)
{
    if((hex >= 0) &&(hex <= 9))
        hex += 0x30;
    else if((hex >= 10) &&(hex <= 15)) //A-F
        hex += 0x41;//0x37
    else
        hex = 0xff;
    return hex;
}

//��������HexToAsc()
//������������16����ת��ΪASCII
int8_t HexToAsc(int8_t aChar)
{
    if((aChar>=0x30) &&(aChar<=0x39))
        aChar -= 0x30;
    else if((aChar>=0x41) && (aChar<=0x46))//��д��ĸ
        aChar -= 0x41; //0x37
    else if((aChar>=0x61) && (aChar<=0x66))//Сд��ĸ
        aChar -= 0x61; //0x57
    else
        aChar = 0xff;
    return aChar;
}

void HexArrayToString(uint8_t *hexarray,uint8_t length,uint8_t *string)
{
    int i = 0;
    while(i < length)
    {
        *(string++) = Num2CharTable[((hexarray[i] >> 4) & 0x0f)-1];
        *(string++) = Num2CharTable[(hexarray[i] & 0x0f)-1];
        i++;
    }
    *string = 0x0;
}
void HexToDsp(uint8_t *hex, uint8_t *dsp, uint8_t hex_count)

{

    int i;

    char ch;

    for(i = 0; i < hex_count; i++)

    {

        ch = (hex[i] & 0xf0) >> 4;

        dsp[i * 2] = (ch > 9) ? ch + 0x41 - 10 : ch + 0x30;

        ch = hex[i] & 0xf;

        dsp[i * 2 + 1] = (ch > 9) ? ch + 0x41 - 10 : ch + 0x30;

    }

}
void Sdk_Int2Char(uint8_t p_nNum, uint8_t *p_Ch)
{
    if(p_nNum <= 9)
    {
        *p_Ch = p_nNum + '0';
    }
    else
    {
        /*0 - 9 ��ʮ����*/
        *p_Ch = (p_nNum -10) + 'A';
    }
}


/*����16�������ݺͳ���*/
void Sdk_Str2BcdStr(uint8_t *p_Str, uint8_t p_nLen, uint8_t *p_StrBcd)
{
    int i =0, j = 0;

    for(i = 0; i < p_nLen; ++i)
    {
        Sdk_Int2Char((p_Str[i] >> 4)&0x0F, &p_StrBcd[j++]);
        Sdk_Int2Char(p_Str[i] &0x0F, &p_StrBcd[j++]);
    }
    return ;
}

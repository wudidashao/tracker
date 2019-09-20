/*******************************************************************************
 * drv_i2c.c
 *
 *  Created on: 2018Äê1ÔÂ23ÈÕ
 *      Author: vladimir
 *
 ******************************************************************************/
//#include <msp430.h>
//#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "msp430.h"
#include "driverlib.h"
#include "drv_i2c.h"
#include "hdc1000.h"



//*****************************************************************************
// Global Variables
//*****************************************************************************
uint8_t i2c_transmitCounter = 0;    //Variable to store transmit status for I2C
uint8_t *p_i2c_transmitData;        //Pointer to transmit data
uint8_t i2c_receiveCounter = 0;     //Variable to store receive status for I2C
uint8_t *p_i2c_receivedData;        //Pointer to received data
uint16_t    i2c_mode = 0;               //Variable to store i2c mode (tx or rx)
//uint8_t testdata;

volatile eUSCI_status ui8Status;
uint8_t* pData;
uint8_t ui8DummyRead;
uint32_t g_ui32ByteCount;
bool     burstMode = false;
//***** Global Data *****
volatile EUSCI_B_I2C_initMasterParam i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        0,
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS,      // Desired I2C Clock of 400khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD                // Autostop
};

void I2CInitialise(void)
{
    // Configure Pins for I2C
    //Set P1.6 and P1.7 as Secondary Module Function Input.
    /*

    * Select Port 1
    * Set Pin 6, 7 to input Secondary Module Function, (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 + GPIO_PIN7,
        GPIO_SECONDARY_MODULE_FUNCTION
    );

    //Set P1.0 as an output pin.
    /*

     * Select Port 1
     * Set Pin 0 as output
     */
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN0
    );

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = 0x07;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE,
        0x48
        );

    //Set Master in receive mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_MODE
        );

    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);

    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
        EUSCI_B_I2C_NAK_INTERRUPT
        );

    //Enable master Receive interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
        EUSCI_B_I2C_NAK_INTERRUPT
        );
}
bool i2c_write(uint8_t SLAVE_ADDRESS)
{
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = 0x07;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
//  while( EUSCI_B_I2C_masterIsStopSent( EUSCI_B0_BASE ) == EUSCI_B_I2C_SENDING_STOP ) ;    //Delay until transmission completes
//
//  EUSCI_B_I2C_setSlaveAddress( EUSCI_B0_BASE, SLAVE_ADDRESS );        //Set slave address
//  EUSCI_B_I2C_setMode( EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE );    //Set Transmit mode
//  EUSCI_B_I2C_enable( EUSCI_B0_BASE );                                //Enable I2C Module to start operations
//
//  EUSCI_B_I2C_clearInterruptFlag( EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 );   //Clear transmit Interrupt
//  EUSCI_B_I2C_enableInterrupt( EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 );      //Enable transmit Interrupt
//
//  __delay_cycles(50);     //Delay between each transaction
//
//  EUSCI_B_I2C_masterMultiByteSendStart( EUSCI_B0_BASE, *p_i2c_transmitData++ );   //Initiate start and send first character
//  i2c_transmitCounter--;
//
//  __bis_SR_register( LPM0_bits + GIE );   //Enter LPM0 with interrupts enabled

    i2c_mode = EUSCI_B_I2C_getMode(EUSCI_B0_BASE);

    if(i2c_mode != EUSCI_B_I2C_TRANSMIT_MODE)
    {
        EUSCI_B_I2C_setMode(
                EUSCI_B0_BASE,
                EUSCI_B_I2C_TRANSMIT_MODE
        );
    }

    //Delay until I2C module is free
    while( EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE) );

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE,
            SLAVE_ADDRESS
    );

    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);

    //Clear transmit Interrupt
    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
                               EUSCI_B_I2C_RECEIVE_INTERRUPT0
        );
    //Enable transmit Interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0
    );

    //Delay between each transaction
    __delay_cycles(50);

    //Initiate start and send first character
    //EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE,
            //*p_i2c_transmitData++
    //);
    //i2c_transmitCounter--;
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE,
                                         *p_i2c_transmitData++);



    //Enter LPM0 with interrupts enabled
    __bis_SR_register(GIE);
    if(ui8Status == eUSCI_NACK)
    {
        return(-1);
    }
    else
    {
        return(0);
    }
}
bool i2c_read_byte(uint8_t SLAVE_ADDRESS, uint8_t BYTE_COUNT)
{
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = 0x07;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);


    i2c_mode = EUSCI_B_I2C_getMode(EUSCI_B0_BASE);

    if(i2c_mode != EUSCI_B_I2C_RECEIVE_MODE)
    {
        EUSCI_B_I2C_setMode(
                EUSCI_B0_BASE,
                EUSCI_B_I2C_RECEIVE_MODE
        );
    }

    i2c_receiveCounter = BYTE_COUNT;

    //Delay until I2C module is free
    while( EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE) );

    //Set slave address
    EUSCI_B_I2C_setSlaveAddress( EUSCI_B0_BASE, SLAVE_ADDRESS );

    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable( EUSCI_B0_BASE );

    //Clear receive interrupt
    EUSCI_B_I2C_clearInterrupt(
            EUSCI_B0_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0 + EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT
    );

    //Enable receive Interrupt
    EUSCI_B_I2C_enableInterrupt(
            EUSCI_B0_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0 + EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT
    );

    //Delay between each transaction
    __delay_cycles(50);

    //Initiate I2C multibyte receive
    EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
    //EUSCI_B_I2C_masterReceiveMultiByteStart(EUSCI_B0_BASE);

    //Enter LPM0 with interrupts enabled
    __bis_SR_register(GIE);
    if(ui8Status == eUSCI_NACK)
    {
        return(-1);
    }
    else
    {
        return(0);
    }
}

//*****************************************************************************
// Interrupt Service Routines
//*****************************************************************************
#pragma vector = USCI_B0_VECTOR
__interrupt void USCIB0_ISR (void)
{

    switch (__even_in_range(UCB0IV, 0x1E)){
        case 0x00:  break;      // Vector 0: No interrupts, break;
        case 0x02:  break;      // Vector 2: No interrupts, break;
        case 0x04:              // Vector 4: NACKIFG break;
            break;
        case 0x06:  break;      // Vector 6: STT IFG break;
        case 0x08:  break;      // Vector 8: STPIFG break;
        case 0x0a:  break;      // Vector 10: RXIFG3 break;
        case 0x0c:  break;      // Vector 14: TXIFG3 break;
        case 0x0e:  break;      // Vector 16: RXIFG2 break;
        case 0x10:  break;      // Vector 18: TXIFG2 break;
        case 0x12:  break;      // Vector 20: RXIFG1 break;
        case 0x14:  break;      // Vector 22: TXIFG1 break;
        case 0x16:              // Vector 24: RXIFG0 break;
        {
            //testdata = EUSCI_B_I2C_masterReceiveSingle( EUSCI_B0_BASE );
            *p_i2c_receivedData++ = EUSCI_B_I2C_masterReceiveSingle( EUSCI_B0_BASE );   // Get RX data

            i2c_receiveCounter--;

                  if(i2c_receiveCounter==1)
                  {
                      //Initiate end of reception -> Receive byte with NAK
                      *p_i2c_receivedData++ = EUSCI_B_I2C_masterReceiveMultiByteFinish( EUSCI_B0_BASE );   // Get RX data
                      //EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
                  }
            break;
        }
        case 0x18:              // Vector 26: TXIFG0 break;
        {
            i2c_transmitCounter--;
            //Check TX byte counter
            if (i2c_transmitCounter)
            {
                //Initiate send of character from Master to Slave
                EUSCI_B_I2C_masterSendMultiByteNext( EUSCI_B0_BASE, *p_i2c_transmitData++ );

                //Increment TX byte counter
                //i2c_transmitCounter--;
            }
            else
            {
                EUSCI_B_I2C_masterSendMultiByteStop( EUSCI_B0_BASE );
            }
            break;
        }
        case 0x1a:  break;      // Vector 28: BCNTIFG break;
        case 0x1c:  break;      // Vector 30: clock low timeout break;
        case 0x1e:  break;      // Vector 32: 9th bit break;
        default:    break;
    }
}



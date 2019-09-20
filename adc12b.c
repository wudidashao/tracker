/*
 * adc12b.c
 *
 *  Created on: 2018Äê1ÔÂ29ÈÕ
 *      Author: vladimir
 */
#include "driverlib.h"
#include "adc12b.h"
float voltage;
void adc12_b_initialise(void)
{
    //Configure P3.3 -A15
    //GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN3, GPIO_TERNARY_MODULE_FUNCTION);
    //PMM_unlockLPM5();
    //If ref generator busy, WAIT
    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;

    //Select internal ref = 1.2V
    Ref_A_setReferenceVoltage(REF_A_BASE,
                              REF_A_VREF1_2V);

    //Turn on Reference Voltage
    Ref_A_enableReferenceVoltage(REF_A_BASE);
    //Initialize the ADC12B Module
    /*
    * Base address of ADC12B Module
    * Use internal ADC12B bit as sample/hold signal to start conversion
    * USE MODOSC 5MHZ Digital Oscillator as clock source
    * Use default clock divider/pre-divider of 1
    * Not use internal channel
    */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_SMCLK;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_8;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    /*
    * Base address of ADC12B Module
    * For memory buffers 0-7 sample/hold for 64 clock cycles
    * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
    * Disable Multiple Sampling
    */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
      ADC12_B_CYCLEHOLD_16_CYCLES,
      ADC12_B_CYCLEHOLD_4_CYCLES,
      ADC12_B_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
    * Base address of the ADC12B Module
    * Configure memory buffer 15
    * Map input A15 to memory buffer 0
    * Vref+ = AVcc
    * Vref- = AVss
    * Memory buffer 0 is not the end of a sequence
    */
    ADC12_B_configureMemoryParam configureMemoryParam = {0};
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A15;
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);


//    ADC12_B_clearInterrupt(ADC12_B_BASE,
//        0,
//        ADC12_B_IFG0
//        );
//
//    //Enable memory buffer 15 interrupt
//    ADC12_B_enableInterrupt(ADC12_B_BASE,
//      ADC12_B_IE0,
//      0,
//      0);

    }
void adc12b_start_conversion(void)
{
    //Enable/Start sampling and conversion
    /*
     * Base address of ADC12B Module
     * Start the conversion into memory buffer 0
     * Use the single-channel, single-conversion mode
     */
    ADC12_B_startConversion(ADC12_B_BASE,
                            ADC12_B_MEMORY_0,
                            ADC12_B_SINGLECHANNEL);

    //__bis_SR_register(GIE);   // Wait for conversion to complete
    //while(!(ADC12IFGR0 & ADC12IFG0));
    __delay_cycles(80000);
    ADC12IFGR0 &= ~ADC12IFG0;             // Clear interrupt flag
    voltage = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0);
    ADC12_B_disable(ADC12_B_BASE);
    }

/*
 * sampling.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Jeffrey Huang
 *              Ravi Kirschner
 */
#include "inc/tm4c1294ncpdt.h"
#include "sampling.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sysctl_pll.h"
#include "math.h"
#include "driverlib/comp.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "inc/tm4c1294ncpdt.h"

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

#include "driverlib/udma.h"
#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64]; // uDMA control table (global)
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];
volatile uint32_t period,last_count, periodcount;

volatile bool gDMAPrimary = true; // is DMA occurring in the primary channel?

#define PWM_PERIOD 258 // PWM period = 2^8 + 2 system clock cycles
uint32_t gPhase = 0; // phase accumulator
uint32_t gPhaseIncrement = 156981055/2; // phase increment for 17 kHz
#define PWM_WAVEFORM_INDEX_BITS 10
#define PWM_WAVEFORM_TABLE_SIZE (1 << PWM_WAVEFORM_INDEX_BITS)
uint8_t gPWMWaveformTable[PWM_WAVEFORM_TABLE_SIZE] = {0};

//Initializes ADC1 to poll at 1Msps
void ADC1_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0); // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);// in the 0th step, sample channel 3 (AIN3)

    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);

    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
     UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
     UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
     (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);

    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
     UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
     UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
     (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);


    // enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0); // enable the sequence. it is now sampling
    ADCSequenceDMAEnable(ADC1_BASE, 0); // enable DMA for ADC1 sequence 0
    ADCIntEnableEx(ADC1_BASE,ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt
}

void COMP0_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);
    ComparatorRefSet(COMP_BASE,COMP_REF_1_546875V);
    ComparatorConfigure(COMP_BASE, 1, COMP_TRIG_NONE|COMP_INT_RISE|COMP_ASRCP_REF|COMP_OUTPUT_NORMAL);
    // configure GPIO for comparator input C1- at BoosterPack Connector #1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeComparator(GPIO_PORTC_BASE,GPIO_PIN_4);
    // configure GPIO for comparator output C1o at BoosterPack Connector #1 pin 15
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeComparatorOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD1_C1O);

    // configure GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff); // use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff); // use maximum prescale value
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void PWM0_Init(void){
    // use M0PWM1, at GPIO PF1, which is BoosterPack Connector #1 pin 40
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1); // PF1 = M0PWM1
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA,
    GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_PERIOD/2); // initial 50% duty cycle
    PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true); // invert PWM output
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // enable PWM output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0); // enable PWM generator
    // enable PWM interrupt in the PWM peripheral
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);

    int i = 0;
    int pwmval = 127;
    int addToVal = 1;
    for(i = 0; i < PWM_WAVEFORM_TABLE_SIZE;i++) {
        gPWMWaveformTable[i] = pwmval;
        if(pwmval == 255) addToVal = -1;
        else if (pwmval == 0) addToVal = 1;
        pwmval+=addToVal;
    }
}

//ADC Hwi to get data from ADC1 sequence 0
void ADC_ISR(void){
    ADCIntClearEx(ADC1_BASE,ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag
    // Check the primary DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) ==
    UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)
        gDMAPrimary = false; // DMA is currently occurring in the alternate buffer
    }
    // Check the alternate DMA channel for end of transfer, and restart if needed.
    // Also set the gDMAPrimary global.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) ==
    UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)
        gDMAPrimary = true; // DMA is currently occurring in the primary buffer
    }
    // The DMA channel may be disabled if the CPU is paused by the debugger.
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10); // re-enable the DMA channel
    }
}

void Freq_ISR(void) {
    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);
    uint32_t count = TimerValueGet(TIMER0_BASE, TIMER_A);
    period+= (count - last_count) & 0xffffff;
    periodcount++;
    last_count = count;
}

void PWM_ISR(void){
    PWMGenIntClear(PWM0_BASE,PWM_GEN_0,PWM_INT_GEN_0); // clear PWM interrupt flag
    gPhase += gPhaseIncrement;
    // write directly to the Compare B register that determines the duty cycle
    PWM0_0_CMPB_R = 1 + gPWMWaveformTable[gPhase >> (32 - PWM_WAVEFORM_INDEX_BITS)];
}

int32_t getADCBufferIndex(void){
    IArg key;
    int32_t index;
    key = GateHwi_enter(gateHwi0);
    if (gDMAPrimary) { // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
    }
    else { // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
    }
    GateHwi_leave(gateHwi0, key);
    return index;
}

/**
 * Finds and returns the trigger index for main() to draw from.
 * triggerDirection is 0 for down, 1 for up
 * Returns -1 if it doesn't find a trigger.
 */
int getTriggerIndex(int triggerDirection) {
    int i, index;
    int tolerence = 25; //Trigger point can be ADC_OFFSET +- tolerence
    bool dir;

    for(i = 64; i < ADC_BUFFER_SIZE/2; i++) {
        index = ADC_BUFFER_WRAP(getADCBufferIndex()+i);
        if(gADCBuffer[index] >= ADC_OFFSET-tolerence && gADCBuffer[index] <= ADC_OFFSET+tolerence) {
            dir = gADCBuffer[ADC_BUFFER_WRAP(index+3)] > gADCBuffer[ADC_BUFFER_WRAP(index-3)]; //tells what direction the sin wave is going
            if ((triggerDirection && dir) || (!triggerDirection && !dir)) return index;
        }
    }
    return -1;
}

/**
 * Scales voltage to a value that reflects div.
 * div is in mV
 */
int voltageScale(uint16_t voltage, float div) {
    float x = VIN_RANGE * PIXELS_PER_DIV;
    float fScale = x/((1 << ADC_BITS) * div);
    return LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)voltage - ADC_OFFSET));
}

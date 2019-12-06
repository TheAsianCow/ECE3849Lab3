/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include "sampling.h"

#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#define PI 3.14159265358979f //Pi
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

uint32_t cpu_load_count(void);
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;

int trigger = 0; //Trigger index for Waveform Task
int divNumber = 1; //current division scale, 0->3 inclusive
float freq = 0;
uint16_t triggerDir = 1; //direction for trigger, 0 or 1
uint16_t mode = 0; //Current mode (Oscope or FFT) 0 or 1
const char* slope[] = {"DOWN","UP"}; //slope strings to draw
const float divArray[] = {0.1,0.2, 0.5, 1}; //division values
tContext sContext; //Context for DisplayTask
uint16_t ADC_local[128]; //Waveform Task values, Oscope mode
uint16_t scaledWave[128]; //Scaled values to draw in Display Task
uint16_t fft_local[NFFT]; //Waveform Task values, Spectrum mode

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // hardware initialization goes here

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    ADC1_Init();
    COMP0_Init();
    PWM0_Init();
    ButtonInit();

    // initialize timer 3 in one-shot mode for polled timing
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, 120000); // 1 sec interval

    count_unloaded = cpu_load_count();

    IntMasterEnable();
    /* Start BIOS */
    BIOS_start();

    return (0);
}

//Clock to post to ButtonSem every 5ms
void Clock(UArg arg){
    Semaphore_post(ButtonSem);
}

void FreqClock(UArg arg) {
    Semaphore_post(FrequencySem);
}

//UI Task, takes button inputs and makes necessary changes to the display values
void UI_Task(UArg arg0, UArg arg1){
    char data = 'A'; //Data to be read into by mailbox
    while(1){
        Mailbox_pend(ButtonBox, &data, BIOS_WAIT_FOREVER); //Wait for mailbox to be posted to
        switch(data) {
        case 'D': //if down, decrement divNumber
            divNumber = divNumber > 0 ? divNumber-1 : divNumber;
            break;
        case 'U': //if up, increment divNumber
            divNumber = divNumber < 3 ? divNumber+1 : divNumber;
            break;
        case 'T': //if trigger, change the trigger direction
            triggerDir = triggerDir == 1 ? 0 : 1;
            break;
        case 'F': //if fft, change the mode
            mode = mode == 1 ? 0 : 1;
            break;
        }
        Semaphore_post(DisplaySem); //Signal Display Task to change the on-screen display
    }
}

//Display Task, draws waveform on the screen using user options
void Display_Task(UArg arg0, UArg arg1) {
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
    char div_str[10];
    char load_str[30];
    char freq_str[30];
    int i = 0;
    while(1) {
        Semaphore_pend(DisplaySem, BIOS_WAIT_FOREVER); //Waits to be signaled
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(&sContext, ClrBlue); // blue lines
        for(i = -3; i < 4; i++){
            GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX-1,LCD_VERTICAL_MAX/2+i*PIXELS_PER_DIV);
            GrLineDrawV(&sContext, LCD_HORIZONTAL_MAX/2+i*PIXELS_PER_DIV, 0,LCD_VERTICAL_MAX-1);
        }
        if(!mode) { //If Oscope mode
            if(divNumber == 3) { //if divNumber is 3, special string protocol to show V instead of mV
                snprintf(div_str, sizeof(div_str), " 1 V");
            }
            else {
                snprintf(div_str, sizeof(div_str), "%03d mV", (int)(divArray[divNumber]*1000));
            }


            GrContextForegroundSet(&sContext, ClrWhite); // white text
            GrStringDraw(&sContext, "20 us", -1, 4, 0, /*opaque*/ false); //draw time scale
            GrStringDraw(&sContext, div_str, -1, 45, 0, /*opaque*/ false); //draw division scale
            GrStringDraw(&sContext, slope[triggerDir], -1, 100, 0, false); //draw trigger direction

            snprintf(freq_str, sizeof(freq_str), "f = %.3f Hz", freq);
            GrContextForegroundSet(&sContext, ClrWhite); // yellow text
            GrStringDraw(&sContext, freq_str, -1, 0, 110, false);

            snprintf(load_str, sizeof(load_str), "CPU load = %.1f%%", cpu_load*100);
            GrContextForegroundSet(&sContext, ClrWhite); // yellow text
            GrStringDraw(&sContext, load_str, -1, 0, 120, false);
        }
        else { //If Spectrum mode
            GrContextForegroundSet(&sContext, ClrWhite); // white text
            GrStringDraw(&sContext, "20 kHz", -1, 0, 0, false); //draw frequency
            GrStringDraw(&sContext, "20 dB", -1, 45, 0, false); //draw division scale
        }

        //draw waveform
        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
        for(i = 0; i < 127; i++) GrLineDraw(&sContext, i, scaledWave[i], i+1, scaledWave[i+1]);
        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}

//Waveform Task, gets the values from the gADCBuffer to eventually display on the screen
void Waveform_Task(UArg arg0, UArg arg1) {
    int i = 0;
    while(1) {
        Semaphore_pend(WaveformSem, BIOS_WAIT_FOREVER); //Wait to be signaled
        if(!mode) { //If Oscope mode
            trigger = getTriggerIndex(triggerDir); //Get trigger value
            for(i = -64; i < 64; i++) {
                //Get 64 values before and 64 values after trigger
                ADC_local[i+64] = gADCBuffer[ADC_BUFFER_WRAP(trigger+i)];
            }
        }
        else { //If spectrum mode
            for(i = 0; i < NFFT; i++) { //Get newest 1024 values
                fft_local[i] = gADCBuffer[ADC_BUFFER_WRAP(getADCBufferIndex()-NFFT+i)];
            }
        }
        Semaphore_post(ProcessingSem); //Signal Processing Task to process values
    }
}

//Processing task, takes values acquired by Waveform Task and turns them into displayable values.
void Processing_Task(UArg arg0, UArg arg1) {
    int i = 0;
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    while(1) {
        Semaphore_pend(ProcessingSem, BIOS_WAIT_FOREVER); //Wait to be signaled
        if(!mode) { //If Oscope mode
            for(i = 0; i < 128; i++) { //Scale voltages by current division
                scaledWave[i] = voltageScale(ADC_local[i], divArray[divNumber]);
            }
        }
        else { //If spectrum mode
            for (i = 0; i < NFFT; i++) { // generate an input waveform
                in[i].r = fft_local[i]*3.3f/4096.f; // real part of waveform
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            // convert first 128 bins of out[] to dB for display
            for(i = 1; i < 129; i++) { //out[0] is useless
                scaledWave[i-1] = 84-(int)roundf(log10f(sqrtf(out[i].r*out[i].r+out[i].i*out[i].i))*20.f);
            }
        }
        Semaphore_post(DisplaySem); //Signal Display Task to draw all of the values
        Semaphore_post(WaveformSem); //Signal Waveform Task to start the loop over again
    }
}

void Frequency_Task(UArg arg0, UArg arg1) {
    IArg key;
    while(1) {
        Semaphore_pend(FrequencySem, BIOS_WAIT_FOREVER);
        key = GateHwi_enter(gateHwi0);
        freq = gSystemClock/((float)period/periodcount);
        period = 0;
        periodcount = 0;
        GateHwi_leave(gateHwi0, key);
    }
}

uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}


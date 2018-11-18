#include "driverlib.h"
#include <stdint.h>
#include <stdio.h>

// Function declaration
//uint16_t distanceWrap(uint16_t value);
const uint16_t rollover = 65535;
    uint16_t clockCount = 0;
    volatile uint16_t regvalue = 0;

// Timer A configuration struct
const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,            // SMCLK Clock Source
        8,                                    // SMCLK/8
        rollover,
        TIMER_A_TAIE_INTERRUPT_ENABLE,        // Enable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,   // Enable CCR0
        TIMER_A_DO_CLEAR                      // Clear Counter
};

// Timer_A Capture Mode Configuration Parameter
const Timer_A_CaptureModeConfig captureModeConfig1 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,            // CC Register 2
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,  // Rising Edge and falling
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,            // CCIxA Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,                  // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,      // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE                // Output bit value
};

// Timer_A Capture Mode Configuration Parameter
const Timer_A_CaptureModeConfig captureModeConfig2 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_2,            // CC Register 2
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,  // Rising Edge and falling
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,            // CCIxA Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,                  // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,      // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE                // Output bit value
};

static void Delay(uint32_t loop)
{
    volatile uint32_t i;
    for (i = 0 ; i < loop ; i++);
}
volatile uint16_t usNum = 0;

volatile int meas1_1 = 0;
volatile int meas2_1 = 0;
volatile float dist1 = 0;

volatile int meas1_2 = 0;
volatile int meas2_2 = 0;
volatile float dist2 = 0;

int main(void)
{
    /* Stop watchdog timer */
    MAP_WDT_A_holdTimer();

    // Enable floating point math
    MAP_FPU_enableModule();

    // Set up red LED to blink when interrupt entered
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    // Set DCO frequency
    MAP_CS_setDCOFrequency(3E+6); // 3 mHz

    // Initialize other stuffs
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Configuring P2.4 and P5.6 as peripheral input for capture
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4,
        GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);

    // Configuring Up Mode
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);

    // Configuring Capture Mode
    MAP_Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig1);
    MAP_Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig2);

    // Enabling interrupts
    MAP_Interrupt_enableInterrupt(INT_TA0_N);
    MAP_Interrupt_enableInterrupt(INT_TA0_0);
    MAP_Interrupt_enableMaster();

    // Starting the Timer_A0 in up mode
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    //Delay(1.5E+6);
    //MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    while(1) {}
}

double distanceWrap(uint16_t value) {
    if (value < 0) {
        return value + rollover;
    }
    else {
        return value;
    }

}

void TA0_N_IRQHandler(void)
{
    switch(TA0IV)
    {
        case 0x0002:
        {
            int rising  = 0;
            //Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

            if(P2IN&0x10) rising=1; else rising=0;

            if(rising) // Start
            {
                meas1_1 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
                //MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
            else
            {
                meas2_1 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
                //MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);

                dist1 = distanceWrap(meas2_1 - meas1_1)*3*3.1/(58*4);
                printf("Sensor 1: %.3f \n",dist1);
            }
            break;
        }

        case 0x0004:
        {
            int rising  = 0;

            //Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);

            if(P2IN&0x20) rising=1; else rising=0;

            if(rising) // Start
            {
                meas1_2 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
                //MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
            else
            {
                meas2_2 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
                //MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);

                dist2 = distanceWrap(meas2_2 - meas1_2)*3*3.1/(58*4);
                printf("Sensor 2: %.3f \n",dist2);
            }
            break;
        }

    }
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
}

// This interrupt triggers ultrasonic sensor 1 on rollover
void TA0_0_IRQHandler(void) {
    // Clear interrupt flag
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);
    if (usNum == 0) {
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
        Delay(50);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
    else{
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
        Delay(50);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
    }

    if (usNum > 0) {
        usNum = 0;
    }
    else usNum++;
}


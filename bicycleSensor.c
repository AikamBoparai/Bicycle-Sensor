#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <math.h>
/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
int edgeCheck = 0;
int ledsOrBuzzer = 0;
int cityCounty = 0;
int edgeCheck2 = 0;
void buzz(void)
{
    int i;
    int j;
    for(i =0; i < 2; i++)
    {
        for(j = 0; j < 1000; j++)
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
            __delay_cycles(10);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
            __delay_cycles(10);
        }
        __delay_cycles(150000);
    }
}

void red()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
   GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
   GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
   GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
}

void yellow()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
}

void orange()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
}

void green()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
}

void turnOffLEDS()
{
   GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
   GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
   GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
   GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
}

void showDistance(int median)
{
    //Display the median accordingly
    showChar((median % 1000 - median % 100)/100+ '0', pos1);
    showChar((median % 100 - median % 10)/10 + '0', pos2);
    showChar((median % 10) + '0', pos3);
}

void triggerUltraSonic()
{
    //Send out the trigger to the sensor
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN7);
}

void main(void)
{
    //Distance from echo
    int distance;

    //Distance in cm after calculation(conversion)
    int final;

    //Use array to sort through different sensor readings and take median
    int median[9];


    //Turn off interrupts during initialization
    __disable_interrupt();



    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default

    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_LCD();     //Sets up the LaunchPad LCD display
    Init_ULTRASONIC();

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //Set the parameters for the timer
    Timer_A_initContinuousModeParam initContinousParameters ={0};
    initContinousParameters.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initContinousParameters.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initContinousParameters.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initContinousParameters.timerClear = TIMER_A_DO_CLEAR;
    initContinousParameters.startTimer= false;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &initContinousParameters);

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    //Setup for pushbutton 1 (sensor change)
    P1DIR |= BIT4;
    P1REN |= BIT2; // Resistor for P1.2
    P1IES = BIT2;
    P1OUT |= BIT2; // Resistor pulls up
    P1IFG &= ~BIT2; // P1.2 IFG cleared
    P1IE |= BIT2;
    P1IFG &= ~BIT2; // P1.2 IFG cleared

    //Setup for pushbutton 2 (Mode change)
    P2REN |= BIT6; // Resistor for P1.2
    P2IES = BIT6;
    P2OUT |= BIT6; // Resistor pulls up
    P2IFG &= ~BIT6; // P1.2 IFG cleared
    P2IE |= BIT6;
    P2IFG &= ~BIT6; // P1.2 IFG cleared
    displayScrollText("CITY MODE");

    while(1)
        {
            //First check to see if we are in city or country mode
            if(cityCounty == 0)
            {
                //Determine which sensor we are reading from
                if(ledsOrBuzzer == 0)
                {
                    int i;
                    for(i = 0; i < 9; i++)
                    {
                        triggerUltraSonic();

                        //Wait for the echo signal
                        while (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) ==0){}

                        //Once the echo signal is receIved start counting
                        Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);

                        //Keep counting while the signal is high
                        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1)==1){}

                        //Stop the timer when the signal is no longer high
                        Timer_A_stop(TIMER_A1_BASE);

                        //Get the time in microseconds
                        final = Timer_A_getCounterValue(TIMER_A1_BASE);

                        //Convert time according to speed of sound and to cm
                        distance=(int)final*0.01715;

                        //Clear the timer for the next signal
                        Timer_A_clear(TIMER_A1_BASE);
                        __delay_cycles(5000);

                        //If the reading is noisy adjust for it
                            if(distance > 170)
                            {
                                distance = distance - 130;
                            }

                            median[i] = distance;
                        }

                    //Sort the array using quicksort method
                    quickSort(median, 0, 8);

                    showDistance(median[4]);

                    if(median[4] < 7 && median[4] >= 0)
                    {
                        red();
                    }

                    else if(median[4] >= 7 && median[4] <= 20)
                    {
                        yellow();
                    }

                    else if(median[4] > 20 && median[4] <= 35)
                    {
                        orange();
                    }

                    else if(median[4] > 40 && median[4] < 160)
                    {
                        green();
                    }


                }

                else if(ledsOrBuzzer == 1)
                {
                    turnOffLEDS();
                    int i;
                    for(i = 0; i < 9; i++)
                    {
                        triggerUltraSonic();

                        //Wait for Echo
                        while (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) ==0){}

                        //Start counting
                        Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);

                        //Keep counting while the Echo is high
                        while(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3)==1){}

                        //Stop the timer
                        Timer_A_stop(TIMER_A1_BASE);

                        //Perform final calculations for distance
                        final = Timer_A_getCounterValue(TIMER_A1_BASE);
                        distance=(int)final*0.01715;

                        //Clear the timer for the next reading
                        Timer_A_clear(TIMER_A1_BASE);
                        __delay_cycles(5000);

                        //Adjust for noise
                        if(distance > 170)
                        {
                            distance = distance - 130;
                        }

                        median[i] = distance;
                    }

                    //Sort the array of readings
                    quickSort(median, 0, 8);

                    showDistance(median[4]);

                    if(distance < 10)
                    {
                        buzz();
                        buzz();
                        __delay_cycles(350000);
                    }

                    else if(distance > 10 && distance < 20)
                    {
                        buzz();
                        __delay_cycles(350000);
                    }
                }
            }

            else if(cityCounty == 1)
            {
                if(ledsOrBuzzer == 0)
                {
                    int i;

                    //Use a bigger array for country for better accuracy
                    int countyMedian[13];
                    for(i = 0; i < 13; i++)
                    {
                        triggerUltraSonic();

                        //Wait for Echo
                        while (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) ==0){}

                        //Start counting
                        Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);

                        //Keep counting while Echo is high
                        while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1)==1){}
                        //Stop counting
                        Timer_A_stop(TIMER_A1_BASE);

                        //Perform calculations for distance
                        final = Timer_A_getCounterValue(TIMER_A1_BASE);
                        distance=(int)final*0.01715;
                        Timer_A_clear(TIMER_A1_BASE);
                        __delay_cycles(5000);

                        //Adjust for noise
                        if(distance > 160)
                        {
                            distance = distance - 80;
                        }

                            median[i] = distance;
                        }

                    //sort the array
                    quickSort(median, 0, 12);


                    showDistance(median[6]);

                    if(median[6] < 15 && median[6] >= 0)
                    {
                        red();
                    }

                    else if(median[6] >= 15 && median[6] <= 40)
                    {
                        yellow();
                    }

                    else if(median[6] > 40 && median[6] <= 75)
                    {
                        orange();
                    }

                    else if(median[6] > 75 && median[6] < 160)
                    {
                        green();
                    }


                }

                else if(ledsOrBuzzer == 1)
                {
                   turnOffLEDS();
                    int i;
                    for(i = 0; i < 9; i++)
                    {
                        triggerUltraSonic();
                        //Wait for Echo
                        while (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3) ==0){}

                        //Start counting
                        Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_CONTINUOUS_MODE);

                        //Keep counting while Echo is high
                        while(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN3)==1){}

                        //Stop counting
                        Timer_A_stop(TIMER_A1_BASE);

                        //Perform calculations for distance
                        final = Timer_A_getCounterValue(TIMER_A1_BASE);
                        distance=(int)final*0.01715;
                        Timer_A_clear(TIMER_A1_BASE);
                        __delay_cycles(5000);

                        //Adjust for noise
                        if(distance > 170)
                        {
                            distance = distance - 130;
                        }

                        median[i] = distance;
                    }

                    //Sort the array
                    quickSort(median, 0, 8);

                    showDistance(median[4]);

                    if(distance < 25)
                    {
                        buzz();
                        buzz();
                        __delay_cycles(350000);
                    }

                    else if(distance > 25 && distance < 40)
                    {
                        buzz();
                        __delay_cycles(350000);
                    }
                }
            }


        }

    /*
     * You can use the following code if you plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

}

/*void Init_TimerA(void)
{
    Timer_A_initContinousModeParam initContParam = {0};
}*/


void swap(int* a, int* b)
{
    int t = *a;
    *a = *b;
    *b = t;
}

int partition (int arr[], int low, int high)
{
    int pivot = arr[high];    // pivot
    int i = (low - 1);  // Index of smaller element
    int j;
    for (j = low; j <= high- 1; j++)
    {
        // If current element is smaller than the pivot
        if (arr[j] < pivot)
        {
            i++;    // increment index of smaller element
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[high]);
    return (i + 1);
}

void quickSort(int arr[], int low, int high)
{
    if (low < high)
    {
        /* pi is partitioning index, arr[p] is now
           at right place */
        int pi = partition(arr, low, high);

        // Separately sort elements before
        // partition and after partition
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}

void Init_ULTRASONIC(void){
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN3);
}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{

    switch( P1IV )     // Clears highest pending flag
          {
            case 6:          // P1.2
            {
                if(edgeCheck == 0)
                {
                    P1IFG &= ~BIT2; // P1.2 IFG cleared
                    if(ledsOrBuzzer == 1)
                    {
                        ledsOrBuzzer = 0;
                        displayScrollText("BACK SENSOR");
                    }

                    else if(ledsOrBuzzer == 0)
                    {
                        ledsOrBuzzer = 1;
                        displayScrollText("FRONT SENSOR");
                    }
                    edgeCheck = 1;


                }

                else
                {
                    edgeCheck = 0;

                }

                break;


            }

            default: break;
          }
}

#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    switch(P2IV)
    {
        case 14:
        {
            if(edgeCheck2 == 0)
            {
                P2IFG &= BIT6;
                if(cityCounty == 0)
                {
                    cityCounty = 1;
                    displayScrollText("COUNTRY MODE");
                }

                else if(cityCounty == 1)
                {
                    cityCounty = 0;
                    displayScrollText("CITY MODE");
                }

                edgeCheck2 = 1;
            }

            else if(edgeCheck2 == 1)
            {
                edgeCheck2 = 0;
            }

            break;
        }

        default:
        {
           break;
        }

    }
}
void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}



/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

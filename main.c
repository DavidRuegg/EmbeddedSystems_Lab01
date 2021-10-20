/**
 * @file    main.c
 *
 * @author  Alice   Guntli
 * @author  David   RUEGG
 *
 * @date    20.10.2021
 *
 * @brief   Control a servomotor through a PWM signal which is generated through the ADC
 *              of a joystick's value.
 */

#include "msp.h"

#define RESOLUTION_14BIT 0x3FFF // 16383
#define TIMER_A_FCLOCK 3000000  // 3MHz
#define PWM_PERIOD 60000        // 3e6 * 0.02 ms = 60000
#define PWM_MIN 3000            // 3e6 * 0.001 ms = 3000
#define PWM_MAX 6000            // 3e6 * 0.002 ms = 6000


void TA0_0_IRQHandler(void){
    ADC14->CTL0 |= ADC14_CTL0_SC;                   // Set bit SC to start ADC14 conversion
    TIMER_A0->CCTL[0] &= (~TIMER_A_CCTLN_CCIFG);    // Reset flag TA0.0
}

void ADC14_IRQHandler(){
    uint16_t joystick_val = ADC14->MEM[0] & RESOLUTION_14BIT;
    //uint32_t pwm_width = joystick_val*(PWM_MAX-PWM_MIN);
    TIMER_A0->CCR[1] = (joystick_val*(PWM_MAX-PWM_MIN)/RESOLUTION_14BIT)+PWM_MIN-1;
}


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // *** CONFIGURE ADC: START ***
    // Configure ADC CTL0 register
    ADC14->CTL0 &= ~(ADC14_CTL0_ENC);       // Disable ENC in order to configure many registers
    ADC14->CTL0 |= ADC14_CTL0_SHS_0;        // Trigger for starting conversion from ADC14SC bit
    ADC14->CTL0 |= ADC14_CTL0_SHP;          // SAMPCON signal is triggered from sampling timer
    ADC14->CTL0 |= ADC14_CTL0_SSEL__MCLK;   // Source clock MCLK-3MHz for conversion
    ADC14->CTL0 |= ADC14_CTL0_CONSEQ_0;     // Mode single channel, single conversion
    ADC14->CTL0 |= ADC14_CTL0_ON;           // Enable ADC core for conversion

    // Configure ADC MCTL0 register for input from pin P4.0-->A13
    ADC14->MCTL[0] |= ADC14_MCTLN_VRSEL_0;  // VR(+)=AVCC VR(-)=AVSS
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_13;  // Configure A13 as input
    ADC14->MCTL[0] |= ADC14_MCTLN_EOS;      // End of sequence after MEM0
    P4->SELC |= BIT0;                       // P4.0 as tertiary function (A13)
    P4->DIR &= ~(BIT0);                     // P4.0 as input

    // Configure ADC interrupt 0 register
    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable interrupt ADC14MEM0

    ADC14->CTL0 |= ADC14_CTL0_ENC;          // Enable conversion when ADC14SC bit is triggered
    // *** CONFIGURE ADC: END ***


    // *** CONFIGURE TIMER A0 for PWM: START ***
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK;            // Select clock SMCLK-3MHz for TA0

    TIMER_A0->CCR[0] = PWM_PERIOD-1;                    // 20 ms
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;            // Interrupt enable for CCR0

    TIMER_A0->CCR[1] = ((PWM_MAX + PWM_MIN)/2)-1;       // 1.5 ms
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_6;        // Mode toggle/set for pwm

    TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;                // Start timer, up to CCR0 mode
    // *** CONFIGURE TIMER A0 for PWM: END ***


    // *** CONFIGURE PIN FOR SERVOMOTR: START ***
    P2->SEL0 |= BIT4;       // P2.4 as secondary function (output from TimerA0.1)
    P2->DIR |= BIT4;        // P2.4 as output
    // *** CONFIGURE PIN FOR SERVOMOTR: END ***


    NVIC_SetPriority(ADC14_IRQn,2);
    NVIC_EnableIRQ(ADC14_IRQn);                 // Enable interrupt ADC14
    NVIC_SetPriority(TA0_0_IRQn,3);
    NVIC_EnableIRQ(TA0_0_IRQn);               // Enable interrupt for TimerA0.0

    while(1){
        __asm__ volatile ("nop");
    }
}














/*** TASK 3 - START ***
void TA0_0_IRQHandler(void){
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // reset flag
    P1->OUT ^= 0x01; // toggle led
}


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    //LED1 on P1.0
    P1->DIR |= 0x01;

    // TIMER A0 Configuration
    TIMER_A0->CCR[0] = 1000; // 1000 ms
    TIMER_A0->EX0 = TIMER_A_EX0_IDEX__4;
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;
    NVIC_EnableIRQ(TA0_0_IRQn);
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__ACLK + TIMER_A_CTL_ID__8 + TIMER_A_CTL_MC__UP;

    while(1){
        __asm__ volatile ("nop");
    }
}
*** TASK 3 - END ***/

/*** TASK 1 -- START ***
// LED2 on P2.0, P2.1, P2.2
P2DIR = 0b0111; // Reset and Set LEDs in output mode
P2OUT = 0; // Reset


while(1){
    P2OUT ^= 0b0001;
    for(delay = 0; delay < SystemCoreClock/16; delay++){
        __asm__ volatile ("nop");
    }
}
 *** TASK 1 -- END ***/

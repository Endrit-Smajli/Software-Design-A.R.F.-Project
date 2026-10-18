
#include <tm4c123gh6pm.h>
#include <losh-libs/nvic.h>
#include "animation.h"

void initGpt0()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
    _delay_cycles(3);

    TIMER0_CTL_R = 0;
    TIMER0_CFG_R = 0;
    TIMER0_TAMR_R = 0x2 | TIMER_TAMR_TAMIE; // enable interrupt.
    TIMER0_TAILR_R = 40000; // 1 millisecond
    TIMER0_TAMATCHR_R = 0;
    TIMER0_IMR_R = TIMER_IMR_TAMIM;
    TIMER0_CTL_R = 1; // enable

    enableNvicInterrupt(INT_TIMER0A);
}

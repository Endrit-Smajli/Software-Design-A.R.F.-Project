
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "pwm.h"

#define OFS_ENABLE 0x8

// Module Reg offsets
#define OFS_CTL_LOAD 0x10
#define OFS_CTL_GENA 0x20
#define OFS_CTL_GENB 0x24
#define OFS_CTL_CMPA 0x18
#define OFS_CTL_CMPB 0x1C

void pwmEnableModules(uint8_t clk_div)
{
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0 | SYSCTL_RCGCPWM_R1; // Just turn on both.
    if(SYSCTL_RCC_PWMDIV_M & clk_div)
        SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | (SYSCTL_RCC_PWMDIV_M & clk_div);
    else
        SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | (SYSCTL_RCC_PWMDIV_M & (clk_div << 17));
    _delay_cycles(3);
}

void pwmSetup(PWM_MOD mod, PWM_GEN gen, uint16_t load)
{
    uint32_t* p;
    p = (uint32_t*)(mod + gen + OFS_CTL_GENA);
    *p |= 0xC8; // Goes low on LOAD, high on CMPA.
    p = (uint32_t*)(mod + gen + OFS_CTL_GENB);
    *p |= 0xC08; // Goes low on LOAD, high on CMPA.
    p = (uint32_t*)(mod + gen + OFS_CTL_LOAD);
    *p |= load;
    p = (uint32_t*)(mod + gen + OFS_CTL_CMPA);
    *p |= 937; // Default value.
    p = (uint32_t*)(mod + gen + OFS_CTL_CMPB);
    *p |= 937; // Default value.
    p = (uint32_t*)(mod + gen);
    *p |= PWM_0_CTL_ENABLE | PWM_0_CTL_DEBUG;
}

void pwmSetCmp(PWM_MOD mod, PWM_GEN gen, PWM_SIG sig, uint16_t val)
{
    uint32_t* p;

    if(sig == PWM_SIGA)
        p = (uint32_t*)(mod + gen + OFS_CTL_CMPA);
    else
        p = (uint32_t*)(mod + gen+ OFS_CTL_CMPB);

    *p = val;
}

void pwmEnableOutput(PWM_MOD mod, PWM_ENABLE_PIN pin, bool on)
{
    uint32_t* p;
    p = (uint32_t*)(mod + OFS_ENABLE);

    if(on)
        *p |=  (1 << pin);
    else
        *p &= ~(1 << pin);
}

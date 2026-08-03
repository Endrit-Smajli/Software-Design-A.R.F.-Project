
#ifndef _PWM_H
#define _PWM_H

#include <stdint.h>
#include <stdbool.h>

typedef enum _PWM_MODULE
{
    PWM_MOD0 = 0x40028000,
    PWM_MOD1 = 0x40029000
} PWM_MOD;

typedef enum _PWM_GEN
{
    PWM_GEN0 = 0x040,
    PWM_GEN1 = 0x080,
    PWM_GEN2 = 0x0C0,
    PWM_GEN3 = 0x100
} PWM_GEN;

typedef enum _PWM_SIG
{
    PWM_SIGA = 0,
    PWM_SIGB
} PWM_SIG;

typedef enum _PWM_ENABLE_PIN
{
//  PWM_PIN_GEN_SIG
    PWM_PIN_0_A = 0,
    PWM_PIN_0_B,
    PWM_PIN_1_A,
    PWM_PIN_1_B,
    PWM_PIN_2_A,
    PWM_PIN_2_B,
    PWM_PIN_3_A,
    PWM_PIN_3_B,
} PWM_ENABLE_PIN;

void pwmEnableModules(uint8_t clk_div);

void pwmSetup(PWM_MOD mod, PWM_GEN gen, uint16_t load);

void pwmSetCmp(PWM_MOD mod, PWM_GEN gen, PWM_SIG sig, uint16_t val);

void pwmEnableOutput(PWM_MOD mod, PWM_ENABLE_PIN pin, bool on);

#endif

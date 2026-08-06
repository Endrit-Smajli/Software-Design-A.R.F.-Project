/*
 * servos.c
 *
 *  Created on: Aug 6, 2026
 *      Author: taylor
 */

#include "losh-libs/gpio.h"
#include "pwm.h"
#include "servo.h"

Servo servos[] =
{
   { // Forward left hip
       false,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN0,
       PWM_SIGA
   },
   { // Forward left knee
       false,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN0,
       PWM_SIGB
   },
   { // Forward right hip
       true,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN1,
       PWM_SIGA
   },
   { // Forward right knee
       true,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN1,
       PWM_SIGB
   },
   { // Back right hip
       true,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN2,
       PWM_SIGA
   },
   { // Back right knee
       true,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN2,
       PWM_SIGB
   },
   { // Back left hip
       false,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN3,
       PWM_SIGA
   },
   { // Back left knee
       false,
       0,
       0,
       1500,
       PWM_MOD0,
       PWM_GEN3,
       PWM_SIGB
   }
};

void servoInitialize()
{
    enablePort(PORTB);
    enablePort(PORTE);
    enablePort(PORTC);

    setPinAuxFunction(PORTB, 6, 4);
    selectPinPushPullOutput(PORTB, 6);
    setPinAuxFunction(PORTB, 7, 4);
    selectPinPushPullOutput(PORTB, 7);

    setPinAuxFunction(PORTB, 4, 4);
    selectPinPushPullOutput(PORTB, 4);
    setPinAuxFunction(PORTB, 5, 4);
    selectPinPushPullOutput(PORTB, 5);

    setPinAuxFunction(PORTC, 4, 4);
    selectPinPushPullOutput(PORTC, 4);
    setPinAuxFunction(PORTC, 5, 4);
    selectPinPushPullOutput(PORTC, 5);

    setPinAuxFunction(PORTE, 4, 4);
    selectPinPushPullOutput(PORTE, 4);
    setPinAuxFunction(PORTE, 5, 4);
    selectPinPushPullOutput(PORTE, 5);

    pwmEnableModules(7);
    pwmSetup(PWM_MOD0, PWM_GEN0, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_0_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_0_B, true);

    pwmSetup(PWM_MOD0, PWM_GEN1, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_1_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_1_B, true);

    pwmSetup(PWM_MOD0, PWM_GEN2, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_2_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_2_B, true);

    pwmSetup(PWM_MOD0, PWM_GEN3, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_3_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_3_B, true);
}

void servoSetPulseWidth(Servo* s, uint32_t us)
{
    if(s->flip)
        us = s->center - (us - s->center);

    if(us < 600 || us > 2400)
        us = s->center;

    uint32_t cycles = (us*10) / 16; // 1 cycle is 1.6us.
                                    // So, to keep it integer math we multiply by 10.
    pwmSetCmp(s->mod, s->gen, s->sig, cycles);
}

void servoSetAngle(Servo* s, int8_t angle) // -128 to 127 degrees
{
    // 0 degrees is 1500us
    // 0.15 degrees a us
    uint16_t us = (angle * 100) / 15 + s->center; // 15 = .15 * 100

    if(s->flip)
        us = s->center - (us - s->center);

    if(us < 600 || us > 2400) // Make sure it's a valid pulse width.
        us = s->center;

    uint32_t cycles = (us*10) / 16; // 1 cycle is 1.6us.
                                    // So, to keep it integer math we multiply by 10.
    pwmSetCmp(s->mod, s->gen, s->sig, cycles);
}

void servoSetPos(Servo* s, float x, float y)
{
//    uint16_t L1 = 9; // 9 half inches
//    uint16_t L2 = 9;
//    // al = arccos((x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2))
//    uint32_t al = fast_acos((x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2));
//
//    // K1 = L1 + L2*cos(al)
//    uint32_t K1 = L1 + (L2 * fast_cos(al)) / SCALE;
//    // K2 = L2 * sin(al)
//    uint32_t K2 = (L2 * fast_sin(al)) / SCALE;
//    // au = atan2(y, x) - atan2(K2, K1)
//    uint32_t au = fast_atan2(y, x) - fast_atan2(K2, K1);


}

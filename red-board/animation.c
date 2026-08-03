/*
 * animation.c
 *
 *  Created on: Aug 1, 2026
 *      Author: taylor
 */

#include <stdint.h>
#include <stdbool.h>

#include "tm4c123gh6pm.h"
#include "animation.h"
#include "pwm.h"


void servoSetPulseWidth(Servo* servo, uint16_t cycles)
{
    servo->targetAngle = (servo->flip ?
            (servo->offset - (cycles - servo->offset)) :
            (cycles));
    pwmSetCmp(servo->mod, servo->gen, servo->sig, servo->targetAngle);

}

void servoSetPulseWidthUs(Servo* s, uint32_t us)
{
    if(s->flip)
        us = s->offset - (us - s->offset);
    if(us < 600 || us > 2400)
        us = s->offset;
    uint32_t cycles = (us*10) / 16;

    pwmSetCmp(s->mod, s->gen, s->sig, cycles);

}

void playAnimation(Animation animation)
{

}

void updateAnimation(Animation ani)
{

}

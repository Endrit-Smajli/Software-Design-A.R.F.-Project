/*
 * animation.h
 *
 *  Created on: Aug 1, 2026
 *      Author: taylor
 */

#ifndef ANIMATION_H_
#define ANIMATION_H_

#include <stdint.h>
#include <stdbool.h>
#include "pwm.h"

typedef struct _Servo
{
    bool flip;
    uint16_t speed;
    uint16_t targetAngle;
    uint16_t offset;

    PWM_MOD mod;
    PWM_GEN gen;
    PWM_SIG sig;
} Servo;

typedef struct _Frame
{
    uint8_t servo;
    uint16_t angle;
    uint32_t time;
} Frame;

typedef struct _Animation
{
    uint16_t speed;
    uint16_t frame;
    uint16_t nFrames;
    Frame* frames;
    Servo* servos[];
} Animation;

void servoSetPulseWidth(Servo* servo, uint16_t cycles);
void playAnimation(Animation animation);
void servoSetPulseWidthUs(Servo* s, uint32_t us);

#endif /* ANIMATION_H_ */

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
#include "math_table.h"

#define SCALE 32767

int32_t ipow(int32_t base, uint32_t exp)
{
    int32_t result = 1;

    while (exp)
    {
        if (exp & 1)
            result *= base;

        exp >>= 1;

        if (exp)
            base *= base;
    }

    return result;
}

int16_t fast_sin(int angle)
{
    angle %= 360;
    if (angle < 0)
        angle += 360;

    if (angle <= 90)
        return sin_table[angle];

    if (angle <= 180)
        return sin_table[180 - angle];

    if (angle <= 270)
        return -sin_table[angle - 180];

    return -sin_table[360 - angle];
}

uint16_t fast_acos(int16_t value)
{
    int32_t index =
        ((int32_t)(value + SCALE) * ACOS_SIZE) /
        (2 * SCALE);

    return acos_table[index];
}

int16_t fast_cos(int angle)
{
    return fast_sin(angle + 90);
}

uint16_t fast_atan2(int32_t y, int32_t x)
{
    if (x == 0)
    {
        if (y > 0) return 90;
        if (y < 0) return 270;
        return 0;
    }

    uint32_t ax = (x >= 0) ? x : -x;
    uint32_t ay = (y >= 0) ? y : -y;

    uint16_t angle;

    if (ax >= ay)
        angle = atan_table[(ay * ATAN_SIZE) / ax];
    else
        angle = 90 - atan_table[(ax * ATAN_SIZE) / ay];

    if (x >= 0 && y >= 0)
        return angle;

    if (x < 0 && y >= 0)
        return 180 - angle;

    if (x < 0 && y < 0)
        return 180 + angle;

    return 360 - angle;
}

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

void servoSetPos(Servo* s, uint16_t x, uint16_t y)
{
    uint16_t L1 = 9; // 9 half inches
    uint16_t L2 = 9;
    // al = arccos((x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2))
    uint32_t al = fast_acos((x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2));

    // K1 = L1 + L2*cos(al)
    uint32_t K1 = L1 + (L2 * fast_cos(al)) / SCALE;
    // K2 = L2 * sin(al)
    uint32_t K2 = (L2 * fast_sin(al)) / SCALE;
    // au = atan2(y, x) - atan2(K2, K1)
    uint32_t au = fast_atan2(y, x) - fast_atan2(K2, K1);
}

void playAnimation(Animation animation)
{

}

void updateAnimation(Animation ani)
{

}

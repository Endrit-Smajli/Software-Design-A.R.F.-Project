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

typedef struct _Frame
{
    uint8_t servo;
    int8_t angle;
    uint32_t time;
} Frame;

typedef struct _Animation
{
    uint16_t speed;
    uint16_t frame;
    uint16_t nFrames;
    Frame* frames;
    uint32_t elapsedTime;
} Animation;

void playAnimation(Animation* animation);
void updateAnimation();

#endif /* ANIMATION_H_ */

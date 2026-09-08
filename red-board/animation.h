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

typedef enum {
    FRONT_LEFT_LEG,
    FRONT_RIGHT_LEG,
    BACK_RIGHT_LEG,
    BACK_LEFT_LEG
} Leg;

typedef struct _Frame
{
    Leg leg;
    float x;
    float y;
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

typedef struct
{
    int hipServo;
    int kneeServo;
    float cX;
    float cY;
} LegData;

void playAnimation(Animation* animation);
void updateAnimation();

#endif /* ANIMATION_H_ */

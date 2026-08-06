/*
 * animation.c
 *
 *  Created on: Aug 1, 2026
 *      Author: taylor
 */

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "tm4c123gh6pm.h"
#include "animation.h"
#include "pwm.h"
#include "servo.h"

#define MAX_ACTIVE_ANIMATIONS 4
static Animation* activeAnis[MAX_ACTIVE_ANIMATIONS] = {NULL, NULL, NULL, NULL};

void playAnimation(Animation* ani)
{
    int i;
    for(i = 0; i < MAX_ACTIVE_ANIMATIONS; i++)
    {
        if(activeAnis[i] == NULL) // empty spot
        {
            activeAnis[i] = ani;
            break;
        }
    }
}

void updateAnimation()
{
    uint8_t currentAni;
    for(currentAni = 0; currentAni < 4; currentAni++)
    {
        if(activeAnis[currentAni] == 0)
            continue;

        Animation* cAni = activeAnis[currentAni];

        // loop animation
        if(cAni->frame >= cAni->nFrames)
        {
            cAni->frame = 0; // reset
            cAni->elapsedTime = 0;
        }

        // Is it time?
        Frame* curFrame = &cAni->frames[cAni->frame];
        while(cAni->frame < cAni->nFrames && cAni->elapsedTime >= curFrame->time)
        {
            // Update servo
            servoSetAngle(&servos[curFrame->servo], curFrame->angle);
            // Next frame
            curFrame = &cAni->frames[++cAni->frame];
        }

        cAni->elapsedTime++;
    }
}

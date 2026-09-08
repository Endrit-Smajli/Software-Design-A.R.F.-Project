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

LegData legs[] = {
    {
         .hipServo = FRONT_LEFT_HIP,
         .kneeServo = FRONT_LEFT_KNEE,
         .cX = 0,
         .cY = -6,
    },
    {
         .hipServo = FRONT_RIGHT_HIP,
         .kneeServo = FRONT_RIGHT_KNEE,
         .cX = 0,
         .cY = -6,
    },
    {
         .hipServo = BACK_RIGHT_HIP,
         .kneeServo = BACK_RIGHT_KNEE,
         .cX = 0,
         .cY = -6,
    },
    {
         .hipServo = BACK_LEFT_HIP,
         .kneeServo = BACK_LEFT_KNEE,
         .cX = 0,
         .cY = -6,
    }
};

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

void doKinematic(Leg leg, float x, float y)
{
    float a = 4.5; // upper leg
    float b = 5.25; // Lower leg

    float h  = hypotf(x,y);
    float aB = acosf( (b*b - a*a - h*h) / (2*a*h) );
    float aH = acosf( (h*h - a*a - b*b) / (2*a*b) );
    float aBp = atan2f(x, y);

    float aHip = aB - aBp;
    float aKnee = aH + (aB - aBp);
    float aLever = aKnee + 7*M_PI/6;
    float aServo = aLever - aHip - 7*M_PI/6;

    int16_t iHip = ((x < 0) ? aHip - 2 * M_PI : aHip) * 180/M_PI;
    int16_t iKnee = aServo * 180/M_PI;

    servoSetAngle(&servos[legs[leg].hipServo], iHip);
    servoSetAngle(&servos[legs[leg].kneeServo], iKnee);
    legs[leg].cX = x;
    legs[leg].cY = y;
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
            //servoSetAngle(&servos[curFrame->servo], curFrame->angle);
            doKinematic(curFrame->leg, curFrame->x, curFrame->y);
            // Next frame
            curFrame = &cAni->frames[++cAni->frame];
        }

        cAni->elapsedTime++;
    }
}

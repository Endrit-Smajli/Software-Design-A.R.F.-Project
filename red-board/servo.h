/*
 * servo.h
 *
 *  Created on: Aug 6, 2026
 *      Author: taylor
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

#define FRONT_LEFT_HIP   0
#define FRONT_LEFT_KNEE  1
#define FRONT_RIGHT_HIP  2
#define FRONT_RIGHT_KNEE 3
#define BACK_LEFT_HIP    4
#define BACK_LEFT_KNEE   5
#define BACK_RIGHT_HIP   6
#define BACK_RIGHT_KNEE  7

typedef struct _Servo
{
    bool flip;
    uint16_t speed;
    uint16_t targetAngle;
    uint16_t center;

    PWM_MOD mod;
    PWM_GEN gen;
    PWM_SIG sig;
} Servo;

extern Servo servos[8];

void servoInitialize();
void servoSetPulseWidth(Servo* servo, uint32_t cycles);
void servoSetAngle(Servo* s, int16_t angle);

#endif /* SERVO_H_ */

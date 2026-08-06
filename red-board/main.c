
/*
 * HARDWARE CONFIGURATION
 *
 * SYSTEM CLOCK is 40 MHz
 *
 * ----- PWM -----
 * USEPWMDIV = 1
 * PWMDIV = 0x7 (divide by 64)
 * SYS_CLK / 64 = 625 KHz (625000)
 * 1.6e-6s per cycle
 *
 * We need a 20ms, 50hz period.
 * 625e6 / 50 = 12500 cycles
 *
 * Center 1.5ms 1500us
 * 1.5e-3 / 1.6e-6 = 937.5 cycles
 *
 * ----- SERVOS -----
 * SERVO0 FRONT LEFT HIP
 *  M0PWM0 PB6 (4) GENERATOR 0 pwmA
 *
 * SERVO1 FRONT RIGHT HIP
 *  M0PWM2 PB4 (4) GENERATOR 1 pwmA
 *
 * SERVO2 BACK RIGHT HIP
 *  M0PWM4 PE4 (4) GENERATOR 2 pwmA
 *
 * SERVO3 BACK LEFT HIP
 *  M0PWM6 PC4 (4) GENERATOR 3 pwmA
 *
 * SERVO4 FRONT LEFT KNEE
 *  PB7 (4) GENERATOR 0 pwmB
 *
 * SERVO5 FRONT RIGHT KNEE
 *  PB5 (4) GENERATOR 0 pwmB
 *
 * SERVO6 BACK RIGHT KNEE
 *  PE5 (4) GENERATOR 0 pwmB
 *
 * SERVO7 BACK LEFT KNEE
 *  PC5 (4) GENERATOR 0 pwmB
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "losh-libs/clock.h"
#include "losh-libs/gpio.h"
#include "losh-libs/wait.h"
#include "pwm.h"
#include "animation.h"
#include "servo.h"

// Animation

Frame ang_push[] = {
    { FRONT_LEFT_HIP,  0, 0 },
    { FRONT_RIGHT_HIP, 0, 0 },
    { BACK_LEFT_HIP,   0, 0 },
    { BACK_RIGHT_HIP,  0, 0 },

    { FRONT_LEFT_HIP,  30, 1000 },
    { FRONT_RIGHT_HIP, 30, 2000 },
    { BACK_LEFT_HIP,   30, 3000 },
    { BACK_RIGHT_HIP,  30, 4000 },

    { FRONT_LEFT_HIP,  45, 5000 },
    { FRONT_RIGHT_HIP, 45, 6000 },
    { BACK_LEFT_HIP,   45, 7000 },
    { BACK_RIGHT_HIP,  45, 8000 },


    { FRONT_LEFT_HIP,  0, 10000 },
    { FRONT_RIGHT_HIP, 0, 10000 },
    { BACK_LEFT_HIP,   0, 10000 },
    { BACK_RIGHT_HIP,  0, 10000 },
};

Animation ang_push_ani = {
    .speed = 1,
    .frame = 0,
    .nFrames = (sizeof(ang_push)/sizeof(Frame)),
    .frames = ang_push,
    .elapsedTime = 0,
};

Frame one_servo_test[] = {
    { FRONT_LEFT_HIP,   0,    0 },
    { FRONT_LEFT_HIP,  30, 1000 },
    { FRONT_LEFT_HIP,  45, 2000 },
    { FRONT_LEFT_HIP,  30, 3000 },
    { FRONT_LEFT_HIP,   0, 4000 },
};

Animation one_servo_test_ani = {
    .speed = 1,
    .frame = 0,
    .nFrames = (sizeof(one_servo_test)/sizeof(Frame)),
    .frames = one_servo_test,
    .elapsedTime = 0
};

int main(void)
{
    initSystemClockTo40Mhz();

    servoInitialize();

    playAnimation(&one_servo_test_ani);

    while(1)
    {
        updateAnimation();
        waitMicrosecond(1000);
    }
}

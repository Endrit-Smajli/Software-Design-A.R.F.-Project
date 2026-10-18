
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
#include "losh-libs/uart0.h"
#include "pwm.h"
#include "animation.h"
#include "servo.h"
#include "gpt.h"

Frame stand_still[] = {
    { FRONT_LEFT_LEG,   0,  -7, 0 },
    { FRONT_RIGHT_LEG,  0,  -7, 0 },
    { BACK_RIGHT_LEG,  -2,  -7, 0 },
    { BACK_LEFT_LEG,   -2,  -7, 0 },
};

Animation stand_still_ani = {
    .speed = 1,
    .frame = 0,
    .nFrames = (sizeof(stand_still)/sizeof(Frame)),
    .frames = stand_still,
    .elapsedTime = 0
};

Frame linear_test[] =
{
   { FRONT_LEFT_LEG,  4,   0,    0 },
   { BACK_LEFT_LEG,   4,   0,    0 },

   { FRONT_LEFT_LEG,  4,   1, 1000 },
   { BACK_LEFT_LEG,   3,   0, 1000 },

   { FRONT_LEFT_LEG,  4,  -1, 3000 },
   { BACK_LEFT_LEG,   5,   0, 3000 },

   { FRONT_LEFT_LEG,  4,   0, 4000 },
   { BACK_LEFT_LEG,   4,   0, 4000 },
};

Animation linear_test_ani = {
    .speed = 1,
    .frame = 0,
    .nFrames = (sizeof(linear_test)/sizeof(Frame)),
    .frames = linear_test,
    .elapsedTime = 0
};

Frame walk[] = {
    { FRONT_LEFT_LEG,   0,  -7, 0 },
    { FRONT_RIGHT_LEG,  0,  -7, 0 },
    { BACK_RIGHT_LEG,   0,  -7, 0 },
    { BACK_LEFT_LEG,    0,  -7, 0 },

    { FRONT_LEFT_LEG,   2,   -5, 4000 }, // Forward
    { BACK_RIGHT_LEG,   2,   -5, 4000 },

    { FRONT_RIGHT_LEG,  0,   -7, 4000 }, // Move other legs
    { BACK_LEFT_LEG,    0,   -7, 4000 },

    { FRONT_LEFT_LEG,   0,  -7, 8000 }, // back
    { BACK_RIGHT_LEG,   0,  -7, 8000 },

    { FRONT_LEFT_LEG,   0,  -7, 8000 },
    { FRONT_RIGHT_LEG,  0,  -7, 8000 },
    { BACK_RIGHT_LEG,   0,  -7, 8000 },
    { BACK_LEFT_LEG,    0,  -7, 8000 },
};

Animation walk_ani = {
    .speed = 1,
    .frame = 0,
    .nFrames = (sizeof(walk)/sizeof(Frame)),
    .frames = walk,
    .elapsedTime = 0
};

Frame leg_out[] = {
    { FRONT_RIGHT_LEG, 0, -7, 4000 },
    { FRONT_RIGHT_LEG, 0, -5, 4000 },
//    { FRONT_RIGHT_LEG, 0, -7, 8000 },
};

Animation legout_ani = {
    .speed = 1,
    .frame = 0,
    .nFrames = (sizeof(leg_out)/sizeof(Frame)),
    .frames = leg_out,
    .elapsedTime = 0,
};

int main(void)
{
    initSystemClockTo40Mhz();

    servoInitialize();

    initUart0();
    setUart0BaudRate(115200, 40e6);

    playAnimation(&legout_ani);
    initGpt0();

    while(1)
    {
        //updateAnimation();
        //waitMicrosecond(1000);
        if(kbhitUart0())
            putcUart0(getcUart0());
    }
}



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
 * SERVO0 HIP
 *  M0PWM0 PB6 (4) GENERATOR 0 pwmA
 *
 * SERVO1 HIP
 *  M0PWM2 PB4 (4) GENERATOR 1 pwmA
 *
 * SERVO2 HIP
 *  M0PWM4 PE4 (4) GENERATOR 2 pwmA
 *
 * SERVO3 HIP
 *  M0PWM6 PC4 (4) GENERATOR 3 pwmA
 *
 * SERVO4 KNEE
 *  PB7 (4) GENERATOR 0 pwmB
 *
 * SERVO5 KNEE
 *  PB5 (4) GENERATOR 0 pwmB
 *
 * SERVO6 KNEE
 *  PE5 (4) GENERATOR 0 pwmB
 *
 * SERVO7 KNEE
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

// Servos

#define FRONT_LEFT_HIP   0
#define FRONT_LEFT_KNEE  1
#define FRONT_RIGHT_HIP  2
#define FRONT_RIGHT_KNEE 3
#define BACK_LEFT_HIP    4
#define BACK_LEFT_KNEE   5
#define BACK_RIGHT_HIP   6
#define BACK_RIGHT_KNEE  7

// Hips
Servo s0 = {
    false,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN0,
    PWM_SIGA
};

Servo s1 = {
    true,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN1,
    PWM_SIGA
};

Servo s2 = {
    true,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN2,
    PWM_SIGA
};

Servo s3 = {
    false,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN3,
    PWM_SIGA
};

// Knees

Servo s4 = {
    false,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN0,
    PWM_SIGB
};

Servo s5 = {
    true,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN1,
    PWM_SIGB
};

Servo s6 = {
    true,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN2,
    PWM_SIGB
};

Servo s7 = {
    false,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN3,
    PWM_SIGB
};

Servo* servos[] = {
//  hip, knee
    &s0, &s4, // Front Left
    &s1, &s5, // Front Right
    &s2, &s6, // Back  Right
    &s3, &s7  // Back  Left
};

// Animation

Frame hip_rotate[] = {
    { 0, 1500, 0 },
    { 0, 1510, 200},
    { 0, 1520, 300},
    { 0, 1530, 400},
    { 0, 1540, 500},
    { 0, 1550, 600},
    { 0, 1500, 800}
};

Frame down[] = {
    { 0, 1500,    0},
    { 1, 1500,    0},
    { 0, 1550,  200},
    { 1, 1550,  200},
    { 0, 1600,  300},
    { 1, 1600,  300},
    { 0, 1650,  400},
    { 1, 1650,  400},
    { 0, 1700,  500},
    { 1, 1700,  500},
    { 0, 1750,  600},
    { 1, 1750,  600},
    { 0, 1800,  700},
    { 1, 1800,  700},
    { 0, 1850,  800},
    { 1, 1850,  800},
    { 0, 1900,  900},
    { 1, 1900,  900},
    { 0, 1950, 1000},
    { 1, 1950, 1000},
    { 0, 2000, 1100},
    { 1, 2000, 1100},

    { 0, 1950, 1300},
    { 1, 1950, 1300},
    { 0, 1900, 1400},
    { 1, 1900, 1400},
    { 0, 1850, 1500},
    { 1, 1850, 1500},
    { 0, 1800, 1600},
    { 1, 1800, 1600},
    { 0, 1750, 1700},
    { 1, 1750, 1700},
    { 0, 1700, 1800},
    { 1, 1700, 1800},
    { 0, 1650, 1900},
    { 1, 1650, 1900},
    { 0, 1600, 2000},
    { 1, 1600, 2000},
    { 0, 1550, 2100},
    { 1, 1550, 2100},
    { 0, 1500, 2200},
    { 1, 1500, 2200},

    { 0, 1450, 2300},
    { 1, 1450, 2300},
    { 0, 1400, 2400},
    { 1, 1400, 2400},
    { 0, 1350, 2500},
    { 1, 1350, 2500},
    { 0, 1300, 2600},
    { 1, 1300, 2600},
    { 0, 1250, 2700},
    { 1, 1250, 2700},
    { 0, 1200, 2800},
    { 1, 1200, 2800},
    { 0, 1150, 2900},
    { 1, 1150, 2900},
    { 0, 1100, 3000},
    { 1, 1100, 3000},
    { 0, 1050, 3100},
    { 1, 1050, 3100},
    { 0, 1000, 3200},
    { 1, 1000, 3200},

    { 0, 1050, 3300},
    { 1, 1050, 3300},
    { 0, 1100, 3400},
    { 1, 1100, 3400},
    { 0, 1150, 3500},
    { 1, 1150, 3500},
    { 0, 1200, 3600},
    { 1, 1200, 3600},
    { 0, 1250, 3700},
    { 1, 1250, 3700},
    { 0, 1300, 3800},
    { 1, 1300, 3800},
    { 0, 1350, 3900},
    { 1, 1350, 3900},
    { 0, 1400, 4000},
    { 1, 1400, 4000},
    { 0, 1450, 4100},
    { 1, 1450, 4100},
    { 0, 1500, 4200},
    { 1, 1500, 4200},
};

Frame push[] = {
    { 0, 1500,    0},
    { 1, 1500,    0},
    { 0, 1600, 1000},
    { 1, 1750, 1000},
    { 0, 1700, 2000},
    { 1, 1900, 2000},
    { 0, 1600, 3000},
    { 1, 1750, 3000},
    { 0, 1500, 4000},
    { 1, 1500, 4000},
};

Animation rf_push_ani = {
    1,
    0,
    (sizeof(push)/sizeof(Frame)),
    push,
    {&s0, &s4}
};

Animation lf_push_ani = {
    1,
    0,
    (sizeof(push)/sizeof(Frame)),
    push,
    {&s1, &s5}
};

Animation lb_push_ani = {
    1,
    0,
    (sizeof(push)/sizeof(Frame)),
    push,
    {&s2, &s6}
};

Animation rb_push_ani = {
    1,
    0,
    (sizeof(push)/sizeof(Frame)),
    push,
    {&s3, &s7}
};

Frame walk[] = {
//  Initialize to default standing.
    { 0, 1500,    0 },
    { 1, 1500,    0 },
    { 2, 1500,    0 },
    { 3, 1500,    0 },
    { 4, 1500,    0 },
    { 5, 1500,    0 },
    { 6, 1500,    0 },
    { 7, 1500,    0 },

//    { 0, 2000, 150 },
//    { 4, 2000, 150 },
//    { 0, 1500, 800},
//    { 4, 1500, 800},
//
//    { 1, 2000, 1000},
//    { 5, 2000, 1000},
//    { 1, 1500, 2000},
//    { 5, 1500, 2000},

// Begin walk
// Front-right and back-left move forward (0,1 and 4,5)
    { 1, 1550,  200 }, // up
    { 5, 1550,  200 },
    { 0, 1300,  200 }, // forward
    { 4, 1300,  200 },
    { 1, 1400,  400 }, // down
    { 5, 1400,  400 },
    { 0, 1500,  800 }, // back
    { 4, 1500,  800 },
    { 1, 1500,  800 },
    { 5, 1500,  800 },
};

Animation hip_rotate_ani = {
    1,
    0,
    7,
    hip_rotate,
    {&s0}
};

Animation rf_down_ani = {
    1,
    0,
    (sizeof(down)/sizeof(Frame)),
    down,
    {&s0, &s4}
};

Animation lf_down_ani = {
    1,
    0,
    (sizeof(down)/sizeof(Frame)),
    down,
    {&s1, &s5}
};

Animation rb_down_ani = {
    1,
    0,
    (sizeof(down)/sizeof(Frame)),
    down,
    {&s2, &s6}
};

Animation lb_down_ani = {
    1,
    0,
    (sizeof(down)/sizeof(Frame)),
    down,
    {&s3, &s7}
};

Animation walk_ani = {
    1,
    0,
    (sizeof(walk)/sizeof(Frame)),
    walk,
    {   // Servos
        &s0, &s4,
        &s1, &s5,
        &s2, &s6,
        &s3, &s7
    }
};

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
    1,
    0,
    (sizeof(ang_push)/sizeof(Frame)),
    ang_push
};

uint32_t msTime = 0;

int main(void)
{
    initSystemClockTo40Mhz();

    // Setup SERVO0 Hip PB6
    enablePort(PORTB);
    enablePort(PORTE);
    enablePort(PORTC);

    setPinAuxFunction(PORTB, 6, 4);
    selectPinPushPullOutput(PORTB, 6);
    setPinAuxFunction(PORTB, 7, 4);
    selectPinPushPullOutput(PORTB, 7);

    setPinAuxFunction(PORTB, 4, 4);
    selectPinPushPullOutput(PORTB, 4);
    setPinAuxFunction(PORTB, 5, 4);
    selectPinPushPullOutput(PORTB, 5);

    setPinAuxFunction(PORTC, 4, 4);
    selectPinPushPullOutput(PORTC, 4);
    setPinAuxFunction(PORTC, 5, 4);
    selectPinPushPullOutput(PORTC, 5);

    setPinAuxFunction(PORTE, 4, 4);
    selectPinPushPullOutput(PORTE, 4);
    setPinAuxFunction(PORTE, 5, 4);
    selectPinPushPullOutput(PORTE, 5);

    pwmEnableModules(7);
    pwmSetup(PWM_MOD0, PWM_GEN0, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_0_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_0_B, true);

    pwmSetup(PWM_MOD0, PWM_GEN1, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_1_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_1_B, true);

    pwmSetup(PWM_MOD0, PWM_GEN2, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_2_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_2_B, true);

    pwmSetup(PWM_MOD0, PWM_GEN3, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_3_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_3_B, true);

    Animation* activeAnis[8] = {&ang_push_ani, 0, 0, 0, 0, 0, 0, 0};
    uint8_t currentAni = 0;

    while(1)
    {
        for(currentAni = 0; currentAni < 8; currentAni++)
        {
            if(activeAnis[currentAni] == 0)
                continue;

            Animation* cAni = activeAnis[currentAni];

            // loop animation
            if(cAni->frame >= cAni->nFrames)
            {
                cAni->frame = 0; // reset
                msTime = 0;
            }

            // Is it time?
            Frame* curFrame = &cAni->frames[cAni->frame];
            while(cAni->frame < cAni->nFrames && msTime >= curFrame->time)
            {
                // Update servo
                servoSetAngle(servos[curFrame->servo], curFrame->angle);
                // Next frame
                curFrame = &cAni->frames[++cAni->frame];
            }

        }

        waitMicrosecond(1000);
        msTime++;

    }
}

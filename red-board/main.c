

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
 * SERVO4 KNEE PB7 (4) GENERATOR 0 pwmB
 *
 * SERVO5 KNEE PB5 (4) GENERATOR 0 pwmB
 *
 * SERVO6 KNEE PE5 (4) GENERATOR 0 pwmB
 *
 * SERVO7 KNEE PC5 (4) GENERATOR 0 pwmB
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
    false,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN1,
    PWM_SIGA
};

Servo s2 = {
    false,
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
    false,
    0,
    0,
    1500,
    PWM_MOD0,
    PWM_GEN1,
    PWM_SIGB
};

Servo s6 = {
    false,
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

    { 0, 2000, 1200},
    { 1, 2000, 1200},
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
};

Animation hip_rotate_ani = {
    1,
    0,
    7,
    hip_rotate,
    {&s0}
};

Animation down_ani = {
    1,
    0,
    (sizeof(down)/sizeof(Frame)),
    down,
    {&s0, &s4}
};

uint32_t msTime = 0;

int main(void)
{
    initSystemClockTo40Mhz();

    // Setup SERVO0 Hip PB6
    enablePort(PORTB);
    setPinAuxFunction(PORTB, 6, 4);
    selectPinPushPullOutput(PORTB, 6);
    setPinAuxFunction(PORTB, 7, 4);
    selectPinPushPullOutput(PORTB, 7);

//    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
//    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_64;
//
//    PWM0_0_GENA_R |= 0xC8;
//    PWM0_0_LOAD_R  = 12500;
//    PWM0_0_CMPA_R  = 937;
//    PWM0_0_CTL_R  |= PWM_0_CTL_ENABLE;
//    PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;

    pwmEnableModules(7);
    pwmSetup(PWM_MOD0, PWM_GEN0, 12500);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_0_A, true);
    pwmEnableOutput(PWM_MOD0, PWM_PIN_0_B, true);

    Animation* cAni = &down_ani;

    while(1)
    {
        // Is it time?
        Frame* curFrame = &cAni->frames[cAni->frame];
        while(cAni->frame < cAni->nFrames && msTime >= curFrame->time)
        {
            Servo* s = cAni->servos[curFrame->servo];
            // Update servo
            servoSetPulseWidthUs(s, curFrame->angle);
            // Next frame
            curFrame = &cAni->frames[++cAni->frame];
        }

        waitMicrosecond(1000);
        msTime++;

        // loop animation
        if(cAni->frame >= cAni->nFrames)
        {
            cAni->frame = 0; // reset
            msTime = 0;
        }
    }

}

/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
/*
Motors:
U2:
PWM1a: P9, 14 - EHRPWM1A
DIR1: P9, 12 - GPIO1_28
MOT_STBY: P9, 41 - GPIO0_20
AIN0: P9, 39 - AIN0
M1B: U2 9 M1 2 (OUT -)
M1A: U2 8 M1 1 (OUT +)


U3:
PWM1b: P9, 16 - EHRPWM1B
DIR2: P9, 15 - GPIO1_16
AIN1: P9, 40 - AIN1
M2B: U3 9 M2 2 (OUT -)
M2A: U3 8 M2 1 (OUT +)

GEAR RATIO: 20.4 : 1
ENCODER 48 CPS

NOMINAL WHEEL DIAMETER 84 MM
NOMIMAL WHEEL BASE 205 MM


PWM2a: P8, 19
PWM2b: P8, 13
*/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR                  1 // id of left motor
#define RIGHT_MOTOR                 2 // id of right motor
#define MDIR1_CHIP                  1 // chip of MDIR1 gpio pin
#define MDIR1_PIN                   28//MDIRR1 gpio(CHIP.PIN) P9.12
#define MDIR2_CHIP                  1 // chip of MDIR2 gpio pin
#define MDIR2_PIN                   16 //  MDIRR2 gpio(CHIP.PIN) P9.15
#define MOT_BRAKE_EN            0,20  // gpio0.20  P9.41
#define MOT_1_POL                   -1 // polarity of motor 1
#define MOT_2_POL                   -1 // polarity of motor 2
#define ENC_1_POL                   1 // polarity of encoder 1
#define ENC_2_POL                   1 // polarity of encoder 2
#define MOT_1_CS                    0 // analog in of motor 1 current sense
#define MOT_2_CS                    1 // analog in of motor 2 current sense
#define GEAR_RATIO                  20.4// gear ratio of motor
#define ENCODER_RES                 48 // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER              0.084 // diameter of wheel in meters
#define WHEEL_BASE                  0.205 // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY       0.1 // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY      0.1 // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ            100 // main filter and control loop speed
#define DT                       0.01 // 1/sample_rate
#define PRINTF_HZ                  10 // rate of print loop
#define RC_CTL_HZ                  25 // rate of RC data update
#define RAD_PER_COUNT	   0.00641648 //Radians per count
#define WHEEL_BASE_DIA	(WHEEL_DIAMETER/2)/WHEEL_BASE //For calculating gamma
//#define DSM_DEAD_ZONE              

#endif

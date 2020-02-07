/*******************************************************************************
* mb_motor.h
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

#include "mb_structs.h"

#ifndef MB_MOTOR_H
#define MB_MOTOR_H


// #define MDIR1_CHIP              
// #define MDIR1_PIN                    //MDIRR1 gpio(CHIP.PIN) P9.12
// #define MDIR2_CHIP              
// #define MDIR2_PIN                    //MDIRR2 gpio(CHIP.PIN)
#define MOT_BRAKE_EN_PIN   0,20         // gpio0.20  P9.41
// #define MOT1_CS_PIN                  // analog in of motor 1 current sense
// #define MOT2_CS_PIN                  // analog in of motor 2 current sense
#define MB_MOTOR_DEFAULT_PWM_FREQ 25000

//fuctions
int mb_motor_init();
int mb_motor_init_freq(int pwm_freq_hz);
int mb_motor_disable();
int mb_motor_brake(int brake_en);
int mb_motor_set(int motor, double duty);
int mb_motor_set_all(double duty);
int mb_motor_cleanup();
double mb_motor_read_current(int motor);
int mb_motor_set_torque(int motor, double torque, mb_state_t* mb_state);
int mb_motor_set_torque_all(double torque, mb_state_t* mb_state);

#endif
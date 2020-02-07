/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/


// scp -r ~/balancebot_tor_deepika_nitish/balancebot-f19/ debian@192.168.8.1:~/src/
// scp -r debian@192.168.4.2:~/src/balancebot-f19 ~/balancebot_tor_deepika_nitish/balancebot-f19/
    
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;
float T_S = 2.0;
float W_NL = 37.0;
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
/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){
    
    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){

    //Motor 1 initialized
    rc_pwm_init(1, pwm_freq_hz);
    rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    
    //Motor 2 initialized
    rc_pwm_init(2, pwm_freq_hz);
    rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    
    //Brake pin initialized
    rc_gpio_init(MOT_BRAKE_EN, GPIOHANDLE_REQUEST_OUTPUT);

    //ADC initialized
    rc_adc_init();

    mb_motor_brake(1);

    init_flag = 1;
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }

    rc_pwm_cleanup(1);


    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }

    //Brake pin value set
    rc_gpio_set_value(MOT_BRAKE_EN, brake_en);

   return 0;
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }

    
    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    int direction = 1; //Default motor rotation direction set to FORWARD

    if(duty < 0)       //Direction check to convert to REVERSE
	    direction = 0; 

    if(fabsf(duty)>1)  //Checking duty cycle limits; -1.0 to 1.0
	    duty = 1;

    switch (motor)
    {
	
	    case 1: { // TODO: Change direction if unstable!
		            rc_gpio_set_value(MDIR1_CHIP, MDIR1_PIN, direction);
		            rc_pwm_set_duty(1, 'A', (fabsf(duty)));
		            break;
	            }
	
	    case 2: {
		            rc_gpio_set_value(MDIR2_CHIP, MDIR2_PIN, !direction);
		            rc_pwm_set_duty(1, 'B', (fabsf(duty)));
		            break;
	            }
    }

 
    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    
    //Setting motor duty cycles
    mb_motor_set(1, duty);
    mb_motor_set(2, duty);
 
    
    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    double current;
    current = 0.0;
    
    switch (motor)
    {
        case 1: {
                    current  = rc_adc_read_raw(0)/500.0;
                    break;
                }
    
        case 2: {
                    current = rc_adc_read_raw(1)/500.0;
                    break;
                }
    }

    return current;
}


/*******************************************************************************
* int mb_motor_set_torque(int motor)
* 
*
*******************************************************************************/
int mb_motor_set_torque(int motor, double torque, mb_state_t* mb_state){
    float w = (mb_state->w_right + mb_state->w_left)/2.0;
    float duty = (torque + (T_S/W_NL)*w)/T_S;
    printf("Duty: %f\n",duty );
    mb_motor_set(motor, duty);
    return 0;
}

int mb_motor_set_torque_all(double torque, mb_state_t* mb_state){
    mb_motor_set_torque(1, torque, mb_state);
    mb_motor_set_torque(2, torque, mb_state);
    return 0;
}

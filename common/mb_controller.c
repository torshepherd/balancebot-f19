#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include "rc/math/filter.h"
#include "rc/math/vector.h"

//STATE VARIABLE FILE
FILE* state_variables;
FILE* setpoint_variables;

//INNER LOOP PID CONTROL
rc_filter_t D1 = RC_FILTER_INITIALIZER;
float D1_KP;
float D1_KI;
float D1_KD;
float D1_TF;
float D1_OFFSET;

//OUTER LOOP PID CONTROL
rc_filter_t D2 = RC_FILTER_INITIALIZER;
float D2_KP;
float D2_KI;
float D2_KD;
float D2_TF;
float D2_MAX;

//ROTATION PID CONTROL
rc_filter_t D3 = RC_FILTER_INITIALIZER;
float D3_KP;
float D3_KI;
float D3_KD;
float D3_TF;

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    rc_filter_pid(&D1, D1_KP, D1_KI, D1_KD, D1_TF, DT); //INNER PID

    rc_filter_pid(&D2, D2_KP, D2_KI, D2_KD, D2_TF, DT); //OUTER PID
    
    rc_filter_pid(&D3, D3_KP, D3_KI, D2_KD, D3_TF, DT); //ROTATION PID
    
    rc_filter_enable_saturation(&D2, -D2_MAX, D2_MAX); //WHEEL ANGLE SATURATION
    
    state_variables = fopen("state_variables.csv","w");
    fprintf(state_variables,"Phi, Left_Phi, Left_Encoder, Right_Phi, Right_Encoder, Turn_Angle, Body_Angle, Duty_cycle, Turn_duty, Left_cmd, Right_cmd");

    setpoint_variables = fopen("setpoint_variables.csv","w");
    fprintf(setpoint_variables,"Wheel_Angle, Fwd_Velocity, Body_Angle, Turn_Angle, Turn_Rate, Manual_Ctrl");

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    
    //INNER LOOP PID PARAMETERS
    fscanf(file, "%f", &D1_KP);
    fscanf(file, "%f", &D1_KI);
    fscanf(file, "%f", &D1_KD);
    fscanf(file, "%f", &D1_TF);
    fscanf(file, "%f", &D1_OFFSET);

    //OUTER LOOP PID PARAMETERS
    fscanf(file, "%f", &D2_KP);
    fscanf(file, "%f", &D2_KI);
    fscanf(file, "%f", &D2_KD);
    fscanf(file, "%f", &D2_TF);
    fscanf(file, "%f", &D2_MAX);
    
    //ROTATION PID PARAMETERS
    fscanf(file, "%f", &D3_KP);
    fscanf(file, "%f", &D3_KI);
    fscanf(file, "%f", &D3_KD);
    fscanf(file, "%f", &D3_TF);
    
    fclose(file);

    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* take inputs from the global mb_state
* write outputs to the global mb_state
* this should only be called in the imu call back function, no mutex needed
* return 0 on success
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){

    //ASSIGN WHEEL ANGLE SETPOINT
    mb_setpoints->wheel_angle += mb_setpoints->fwd_velocity*DT/(WHEEL_DIAMETER/2);     

    //ASSIGN XY PLANE ROTATION SETPOINT
    mb_setpoints->gamma += mb_setpoints->turn_rate*DT;

    printf("Setpoint: %f\n", mb_setpoints->wheel_angle);
    printf("State phi: %f\n", mb_state->phi);
    
    //ASSIGN BODY ANGLE SETPOINT FROM OUTER LOOP OUTPUT
    mb_setpoints->theta = rc_filter_march(&D2, (mb_setpoints->wheel_angle) - (mb_state->phi));
    
    //PWM DUTY CYCLE FROM INNER LOOP PID
    double D1_u = rc_filter_march(&D1, (mb_setpoints->theta + D1_OFFSET) - (mb_state->theta)); 

    //PWM DUTY CYCLE FROM ROTATION PID
    double D3_u = rc_filter_march(&D3, (mb_setpoints->gamma) - (mb_state->gamma)); 
    
    //DUTY CYCLE LEFT MOTOR
    mb_state->left_cmd = D1_u - D3_u;
    
    //DUTY CYCLE RIGHT MOTOR
    mb_state->right_cmd = D1_u + D3_u;
    
    
    fprintf(state_variables, "\n %f, %f, %d, %f, %d, %f, %f, %f, %f, %f, %f ",mb_state->phi, mb_state->delta_left_phi, mb_state->left_encoder, mb_state->delta_right_phi, mb_state->right_encoder, mb_state->gamma, mb_state->theta, D1_u, D3_u, mb_state->left_cmd, mb_state->right_cmd);
    fprintf(setpoint_variables,"\n %f, %f, %f, %f, %f, %d",mb_setpoints->wheel_angle, mb_setpoints->fwd_velocity, mb_setpoints->theta, mb_setpoints->gamma, mb_setpoints->turn_rate, mb_setpoints->manual_ctl);

    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*i<3robots!
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    fclose(state_variables);
    rc_filter_free(&D1);
    rc_filter_free(&D2);
    return 0;
}
/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"
#include <math.h>
#include "rc/math/filter.h"
#include "rc/math/vector.h"

rc_filter_t VEL_X = RC_FILTER_INITIALIZER;
rc_filter_t VEL_Y = RC_FILTER_INITIALIZER;
rc_filter_t VEL_Z = RC_FILTER_INITIALIZER;

// rc_filter_t DISP_X = RC_FILTER_INITIALIZER;
// rc_filter_t DISP_Y = RC_FILTER_INITIALIZER;
// rc_filter_t DISP_Z = RC_FILTER_INITIALIZER;


float threshold_gamma = 0.1;

FILE* kinematics;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
/* TODO */
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->gamma = theta;

    //Velocity integrator
    // rc_filter_integrator(&VEL_X,DT);
    // rc_filter_integrator(&VEL_Y,DT);
    // rc_filter_integrator(&VEL_Z,DT);

    //Displacement integrator
    // rc_filter_integrator(&DISP_X,DT);
    // rc_filter_integrator(&DISP_Y,DT);
    // rc_filter_integrator(&DISP_Z,DT);

    kinematics = fopen("kinematics.csv","w");
    fprintf(kinematics,"Acc_x , Acc_y, Acc_z, Vel_x, Vel_y, Vel_z, Disp_x, Disp_y, Disp_z"); 

}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state, mb_setpoints_t *mb_setpoints){
/* TODO */

	
	
	// mb_state->vel_x = rc_filter_march(&VEL_X,mb_state->accel_x);
	// mb_state->vel_y = rc_filter_march(&VEL_Y,mb_state->accel_y);
	// mb_state->vel_z = rc_filter_march(&VEL_Z,mb_state->accel_z);
	// fprintf(kinematics,"%f, %f, %f, %f, %f, %f", mb_state->accel_x, mb_state->accel_y, mb_state->accel_z, mb_state->vel_x, mb_state->vel_y, mb_state->vel_z);
	

    if ((mb_state->yaw*DT - mb_state->delta_gamma)>threshold_gamma){
        mb_state->delta_gamma = mb_state->yaw*DT; 
    }

    mb_odometry->gamma += mb_state->delta_gamma;
    mb_odometry->x += mb_state->delta_phi*(WHEEL_DIAMETER/2)*cos(mb_odometry->gamma);
    mb_odometry->y += mb_state->delta_phi*(WHEEL_DIAMETER/2)*sin(mb_odometry->gamma);
    // printf("\n Odometry X, Y, gamma: %f %f %f \n",mb_odometry->x, mb_odometry->y, mb_odometry->gamma);
    // fclose(kinematics);

}

void mb_odometry_cleanup(){
    rc_filter_free(&VEL_X);
    rc_filter_free(&VEL_Y);
    rc_filter_free(&VEL_Z);
 fclose(kinematics);
}





float mb_clamp_radians(float angle){
    return 0;
}
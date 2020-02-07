/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for (int i = 0; i < count; ++i)
{
     code 
} your motors
*
* TODO: Capture encoder readings, current readings, timestamps etc. to a file
*       to analyze and determine motor parameters
*
*******************************************************************************/
/*******************************************************************************
*Motor Specs from Datasheet:
*---------------------------------------------------
*Gear Ratio             : 20.4 : 1 
*No-Load Speed @ 12 V   :    370 rpm     38.76 rad/s
*No-Load Current @ 12 V :    200 mA      
*Stall Current @ 12 V   :   2100 mA
*Stall Troqure @ 12 V   :     42 oz.in  0.2966 Nm
*Weight                 :     98 g       0.098 kg
*Shaft Diameter 		:      4 mm      0.004 m
*Encoder count			:     48 cps
*Output shaft count		: 979.62 cps
*---------------------------------------------------
*
*Power Bank:
*-----------
*Max V : 12 V
*MAX I :  3 A
*
*Encoder Channel Right Motor: 2
*Encoder Channel Left Motor : 1
*
*Motor Constants as Measured:
*----------------------------
*Resistance: 
*   Left Motor  : 6.1 ohms 
*   Right Motor : 7.0 ohms
*
*******************************************************************************/





#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

FILE* f1;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }


    struct timeval t;

    f1 = fopen("datamotorparams.csv","w");
    int channel_1;
    int channel_2;
    double current_right;
    double current_left;

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
    mb_motor_brake(1);
    mb_motor_set(RIGHT_MOTOR, 0.5);
    mb_motor_set(LEFT_MOTOR, 0.5);

    rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING){
    	
        rc_nanosleep(1E9);
        
        gettimeofday(&t, NULL); 
        
        current_left = mb_motor_read_current(LEFT_MOTOR); 
        current_right = mb_motor_read_current(RIGHT_MOTOR);
        
        channel_1 = rc_encoder_eqep_read(1); //Left Motor
        channel_2 = rc_encoder_eqep_read(2); //Right Motor
        
        fprintf(f1, "%ld %d %d %f %f\n", (1000000*t.tv_sec + t.tv_usec), channel_1, channel_2, current_left, current_right);

    }

	// exit cleanly
    rc_adc_cleanup();
    mb_motor_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}
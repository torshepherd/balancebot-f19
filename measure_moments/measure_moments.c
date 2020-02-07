/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
* 
* TODO: capture the gyro data and timestamps to a file to determine the period.
*
*******************************************************************************/
//207.4 grams divided by 2
//1108.9 grams robot without wheels and hubs
// Orientation 1 : 174 mm Vertical axis  t =  1.9 s
// Orientatoin 2 : 130 mm Horizontal long axis  t = 3.7 s
// Orientation 3 :  74 mm Horizontal short axis   t = 5.7 s 






#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>

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

    rc_mpu_data_t imu_data; //Struct to hold imu data
    rc_mpu_config_t imu_conf = rc_mpu_default_config(); //Default configuration for now
    struct timeval t;

	f1 = fopen("datafile_1.csv","w");

    rc_mpu_initialize(&imu_data, imu_conf); //Initialized IMU

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

    rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING){
        rc_nanosleep(1E6);
        rc_mpu_read_gyro(&imu_data);
        gettimeofday(&t, NULL);
        fprintf(f1,"%ld %f %f %f\n",(1000000*t.tv_sec + t.tv_usec), imu_data.gyro[0] , imu_data.gyro[1], imu_data.gyro[2]);
    }
    fclose(f1);

	// exit cleanly
    rc_mpu_power_off();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}

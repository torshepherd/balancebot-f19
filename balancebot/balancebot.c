/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include "../common/mb_defs.h"
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <inttypes.h>
#include "balancebot.h"

FILE* f1;
float scaler_left;
float scaler_right;
float scaler_wheel_base;

//VELOCITY CONTROLLER
float V_KP;

//TURN CONTROLLER
float W_KA;
float W_KB;

float goal_x;
float goal_y;
/*******************************************************************************
* int main() 
*
*******************************************************************************/
int num_iter = 0;
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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	/*
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};*/

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);
	mb_state.curr_time = rc_nanos_since_boot();
	mb_setpoints.wheel_angle = 0;
    mb_setpoints.fwd_velocity = 0;
    mb_state.phi = 0;
    mb_setpoints.manual_ctl = 1;
    mb_state.num_iter = 0;

    //Scalers for odometry correction
    FILE* scaler_file = fopen(CFG_PATH_SCALER, "r");
    if (scaler_file == NULL){
        printf("Error opening %s\n", CFG_PATH_SCALER);
    }
    fscanf(scaler_file, "%f", &scaler_left);
    fscanf(scaler_file, "%f", &scaler_right);
    fscanf(scaler_file, "%f", &scaler_wheel_base);
    fscanf(scaler_file, "%f", &V_KP);
    fscanf(scaler_file, "%f", &W_KA);
    fscanf(scaler_file, "%f", &W_KB);
    fclose(scaler_file);

	f1 = fopen("datafile_1.csv","w");

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0, 0.0, 0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
    fclose(f1);
    mb_odometry_cleanup();
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}

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
*Encoder count			:     48 cpr
*Output shaft count		: 979.62 cpr
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
/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);

	// Read IMU

	//State update
	mb_state.theta = -mpu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.roll = mpu_data.dmp_TaitBryan[TB_ROLL_Y];
	mb_state.yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z];
	printf("\n roll : %f and yaw : %f \n",mb_state.roll, mb_state.yaw);
	int d_left = rc_encoder_eqep_read(1);
	int d_right = -rc_encoder_eqep_read(2);
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);
	/*
	
	//If not the first iteration, shift previous encoder readings
   	if(mb_state.num_iter>0){
		for(int i=0; i<4; i++){
			mb_state.d_left[4-i] = mb_state.d_left[3-i];
			mb_state.d_right[4-i] = mb_state.d_right[3-i];
		}
	}

	//If the first iteration, initialize all previous encoder readings to the current value
	else{
		for(int i=0; i<4; i++){
			mb_state.d_left[4-i] = d_left;
			mb_state.d_right[4-i] = d_right;
		}
	}

	//Set the 0th element to the current reading to filter
	mb_state.d_left[0] = d_left;
	mb_state.d_right[0] = d_right;

	//Initialize the sorted histories
	int d_left_sorted[5];
	int d_right_sorted[5];

	for(int i=0; i<5; i++){
		d_left_sorted[i] = mb_state.d_left[i];
		d_right_sorted[i] = mb_state.d_right[i];
	}

	//Sort the arrays
	qsort(d_left_sorted, 5, sizeof(int));
	qsort(d_right_sorted, 5, sizeof(int));

	//Set the current filtered encoder value to the median of the sorted list
	mb_state.d_left[0] = d_left_sorted[2];
	mb_state.d_right[0] = d_right_sorted[2];

	mb_state.delta_left_phi = RAD_PER_COUNT*mb_state.d_left[0];
	mb_state.delta_right_phi = RAD_PER_COUNT*mb_state.d_right[0];
	*/
	mb_state.delta_left_phi = RAD_PER_COUNT*d_left*scaler_left;
	mb_state.delta_right_phi = RAD_PER_COUNT*d_right*scaler_right*scaler_left;
	mb_state.delta_phi = (mb_state.delta_left_phi + mb_state.delta_right_phi) / 2;
	mb_state.delta_gamma = (mb_state.delta_right_phi - mb_state.delta_left_phi) * WHEEL_BASE_DIA*scaler_wheel_base;
	mb_state.phi += mb_state.delta_phi;
	mb_state.gamma += mb_state.delta_gamma;
	mb_state.accel_x = mpu_data.accel[1];
	mb_state.accel_y = mpu_data.accel[2];
	mb_state.accel_z = mpu_data.accel[3];



	//State Derived Kinematics Update 



    // Update odometry 
    mb_odometry_update(&mb_odometry, &mb_state, &mb_setpoints);
     // Calculate controller outputs
    //mb_setpoints.manual_ctl = 1;
    if(!mb_setpoints.manual_ctl){
    	// goal_x = goals_x[index];
    	// goal_y = goals_y[index];
    	printf("AUTONOMOUS!!");
    	goal_x = 0.0;
    	goal_y = 0.0;
	    float delta_x = goal_x - mb_odometry.x;
	    float delta_y = goal_y - mb_odometry.y;
	    // if ((abs(delta_x) < 0.001)&&(abs(delta_y) < 0.001)){
	    // 	index += 1;
	    // 	goal_x = goals_x[index];
	    // 	goal_y = goals_y[index];
		   //  delta_x = goal_x - mb_odometry.x;
		   //  delta_y = goal_y - mb_odometry.y;
	    // }
	    float rho = sqrt(delta_x*delta_x + delta_y*delta_y);
	    float alpha = -mb_odometry.gamma + atan2(delta_y,delta_x);
	    float beta = -mb_odometry.gamma - alpha;
	    mb_setpoints.fwd_velocity = V_KP*rho;
	    mb_setpoints.turn_rate = W_KA*alpha + W_KB*beta;
	    if(mb_setpoints.fwd_velocity>0.5){mb_setpoints.fwd_velocity=0.5;}
	    if(mb_setpoints.fwd_velocity>2.0){mb_setpoints.fwd_velocity=2.0;}
	    if(mb_setpoints.fwd_velocity<-2.0){mb_setpoints.fwd_velocity=-2.0;}
 		mb_controller_update(&mb_state, &mb_setpoints);
    	mb_motor_set(LEFT_MOTOR, MOT_1_POL*mb_state.left_cmd);
    	mb_motor_set(RIGHT_MOTOR, MOT_2_POL*mb_state.right_cmd);
   	}

    if(mb_setpoints.manual_ctl){
    	printf("MANUAL!!");
    	//send motor commands, manual
    	// hard-code zero control
    	//mb_setpoints.fwd_velocity = 0;
		//mb_setpoints.turn_rate = 0;
 		mb_controller_update(&mb_state, &mb_setpoints);
       	mb_motor_set(LEFT_MOTOR, MOT_1_POL*mb_state.left_cmd);
    	mb_motor_set(RIGHT_MOTOR, MOT_2_POL*mb_state.right_cmd);
   	}

	// XBEE_getData();
	// double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	// double tb_array[3] = {0, 0, 0};
	// rc_quaternion_to_tb_array(q_array, tb_array);
	// mb_state.opti_x = xbeeMsg.x;
	// mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	// mb_state.opti_roll = tb_array[0];
	// mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	// mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	

	mb_state.num_iter += 1;

   	//unlock state mutex

    pthread_mutex_unlock(&state_mutex);

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	
	double drive_ch;
	double turn_ch;
	float mode_ch;
	int reset_ch;


	while(1){
	 	if(rc_dsm_is_new_data()){
	 		drive_ch = rc_dsm_ch_normalized(1); //DRIVE CHANNEL : 1
	  		turn_ch = rc_dsm_ch_normalized(2); //TURN CHANNEL : 2
	  		mode_ch = rc_dsm_ch_normalized(5); //MODE CHANNEL : 5
	  		reset_ch = rc_dsm_ch_normalized(6);

	  		pthread_mutex_lock(&setpoint_mutex);
			
			if(mode_ch<0){
				//MANUAL MODE
				mb_setpoints.manual_ctl = 1;
				mb_setpoints.fwd_velocity = drive_ch * 1 ;
				mb_setpoints.turn_rate = turn_ch * 3.0;
 				}
	
			else{
				//AUTONOMOUS MODE
				mb_setpoints.manual_ctl = 0;
 				}

			

 			pthread_mutex_unlock(&setpoint_mutex);

 			
	
		}
		rc_nanosleep(1E9 / RC_CTL_HZ);
	}

	
return NULL;
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 
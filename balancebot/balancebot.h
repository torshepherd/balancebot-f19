#ifndef BB_H
#define BB_H
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <string.h>
#include <math.h> // for M_PI
#include <signal.h>
#include <pthread.h>
#include <rc/mpu.h>
#include <rc/math/quaternion.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motor.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"

//#include "../xbee_serial/xbee_receive.h"

//Scaler file path for odometry
#define CFG_PATH_SCALER "dim.cfg"

// global variables
rc_mpu_data_t mpu_data;
pthread_mutex_t state_mutex;
pthread_mutex_t setpoint_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;

//xbee_packet_t xbeeMsg;
int XBEE_portID;

// functions
void balancebot_controller();
void test_printing();

//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);

#endif
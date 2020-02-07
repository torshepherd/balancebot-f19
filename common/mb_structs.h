#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H
#include <inttypes.h>

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   roll;
    float   yaw;
    float   delta_phi;               // average wheel angle (rad)
    float   phi;               // average wheel angle (rad)
    float   delta_left_phi;
    float   delta_right_phi;
    float   delta_gamma;
    float   gamma;
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading
    int     d_left[5];
    int     d_right[5];
    int     left_encoder_prev; // previous left encoder
    int     right_encoder_prev; // previous right encoder

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]
    double   left_torque;  //left wheel command [-1..1]
    double   right_torque; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    // Filtered Sensor info
    float v_robot;
    float w_robot;
    float w_right;
    float w_left;

    //Kinematics
    float accel_x;
    float accel_y;
    float accel_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float disp_x;
    float disp_y;
    float disp_z;

    uint64_t curr_time;
    int num_iter;

    //TODO: Add more variables to this state as needed
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float wheel_angle;
    float theta;
    float gamma;
    float turn_rate; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float gamma;      //orientation from initialization in rad
};

#endif
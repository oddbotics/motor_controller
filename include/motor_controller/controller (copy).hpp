#ifndef MOTOR_CONTROLLER_HPP_
#define MOTOR_CONTROLLER_HPP_

#include <iostream>
#include <cmath>
#include <chrono>
#include <csignal>
#include "../../interface/BlackLib/v2_0/include/BlackPWM.h"
#include "../../interface/BlackLib/v2_0/include/BlackGPIO.h"
#include "../../interface/encoders/include/eqep.h"
#include "PID.hpp"

// PID Parameters for the specific motor
float Kp_vel = 1.0;
float Ki_vel = 1.0;
float Kd_vel = 1.0;
float Kp_pos = 1.0;
float Ki_pos = 1.0;
float Kd_pos = 1.0;
long double time_step_ns = 10000000;
long double time_step_s = 0.01;
float min_vel_output = -100.0;
float max_vel_output = 100.0;
float min_pos_output = -100.0;
float max_pos_output = 100.0;

// motor parameters
float wheel_radius_m = 0.123825/2.0;
float ticks_per_rev = 3200;

// control variables
bool POSE_CONTROL = false;
bool VEL_CONTROL = true;

// desired control 
float desVel_mps = 0.0;
float desPos_m = 0.0;

// current control
float curVel_mps = 0.0;
float curPos_m = 0.0;
float curCur_amp = 0.0;

//main loop control
bool is_okay = true;

#endif

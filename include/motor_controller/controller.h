#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include "oddbot_msgs/MotorCommand.h"
#include "oddbot_msgs/MotorFeedback.h"
#include "sensor_msgs/JointState.h"
#include "motor_controller/PID.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <cstring>
#include <string>
#include "beaglebone_blacklib/BlackPWM.h"
#include "beaglebone_blacklib/BlackGPIO.h"
#include "beaglebone_eqep/eqep.h"

class motor_controller
{
	public:
		motor_controller();
		float update_motor(int deltaEncoder_ticks);
		void update_feedback();
		double getTimeStepS(){return this->time_step_s;}
		std::string getEqepPath(){return this->eqep_path;}
		
	private:
		//call back function for subscriber
		void update_controller(const oddbot_msgs::MotorCommand::ConstPtr& msg);
		
		// PID Parameters for the specific motor
		double Kp_vel;
		double Ki_vel;
		double Kd_vel;
		double Kp_pos;
		double Ki_pos;
		double Kd_pos;
		double time_step_s;
		double min_output;
		double max_output;
				
		// motor parameters
		double wheel_radius_m;
		int ticks_per_rev;
		double max_vel_mps;		

		//control variables
		int mode;
		double timeout;
		
		// desired control 
		double des_vel_mps;
		double des_pos_m;

		// current control
		double cur_vel_mps;
		double cur_pos_m;
		double cur_cur_amp;
		
		//PID handles
		PID vel_PID;
		PID pos_PID;
		
		//eQEP
		std::string eqep_path;
		
		//Subscribers and publishers
		ros::Subscriber command_sub;
		ros::Publisher feedback_pub,joint_pub;
		
		//node name to use for joint state
		std::string node_name;
};

#endif

/*
 * \motor_controller.cpp
 * \controls the motor based on PID
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date February 14, 2015
 */

#include "motor_controller/controller.h"

/** 
 * the constructor for the class motor_controller
 * it is used to initialize the ros node 
 */
motor_controller::motor_controller(){

	//the main node handle
	ros::NodeHandle nh;

	//grab the parameters
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param<double>("kp_vel", Kp_vel, 400);
	private_node_handle_.param<double>("ki_vel", Ki_vel, 5.5);
	private_node_handle_.param<double>("kd_vel", Kd_vel, 0.01);
	private_node_handle_.param<double>("kp_pos", Kp_pos, 1.0);
	private_node_handle_.param<double>("ki_pos", Ki_pos, 1.0);
	private_node_handle_.param<double>("kd_pos", Kd_pos, 1.0);
	private_node_handle_.param<double>("rate_s", time_step_s, 0.01);
	private_node_handle_.param<int>("mode", mode, 1);
	private_node_handle_.param<int>("ticks_per_rev", ticks_per_rev, 3200);
	private_node_handle_.param<double>("wheel_radius_m", wheel_radius_m, 0.0619125);
	private_node_handle_.param<double>("max_vel_mps", max_vel_mps, 2.0);
	private_node_handle_.param<double>("min_output", min_output, -100.0);
	private_node_handle_.param<double>("max_output", max_output, 100.0);
	private_node_handle_.param<std::string>("eqep_path", eqep_path, "/sys/devices/ocp.3/48304000.epwmss/48304180.eqep");
	
	// initialize PID loops
	vel_PID = PID(Kp_vel, Ki_vel, Kd_vel, min_output, max_output);
	pos_PID = PID(Kp_pos, Ki_pos, Kd_pos, min_output, max_output);
  
	//initialize the publishers and subscribers
	node_name = ros::this_node::getName();
	std::string joint("/joint");
        std::string feedback("/feedback");
	std::string command("/command");

	//char *n = new char[node_name.length() + 1];
	//std::strcat(n,node_name.c_str());
	
	std::string joint_name(node_name.append(joint));
	std::string feedback_name(node_name.append(feedback));
	std::string command_name(node_name.append(command));
	
	joint_pub = nh.advertise<sensor_msgs::JointState>(joint_name, 1000);
	feedback_pub = nh.advertise<oddbot_msgs::MotorFeedback>(feedback_name, 1000);
	command_sub = nh.subscribe(command_name, 1000, &motor_controller::update_controller, this);
	
	//intialize state of the controller
	// desired control 
	double des_vel_mps = 0.0;
	double des_pos_m = 0.0;

	// current control
	double cur_vel_mps = 0.0;
	double cur_pos_m = 0.0;
	double cur_cur_amp = 0.0;
	
//	delete[] n;
  
}

/**
 * a call back function that takes in the desired values for the 
 * controller and updates the variables accordingly
 */
void motor_controller::update_controller(const oddbot_msgs::MotorCommand::ConstPtr& msg){
	this->mode = msg->mode;
	this->timeout = msg->timeout;
	// desired control 
	if(fabs(msg->motor_vel) < this->max_vel_mps){ 
		this->des_vel_mps = msg->motor_vel;
	} else {
		this->des_vel_mps = this->max_vel_mps;
	}
	this->des_pos_m = msg->motor_pos;
}

/**
 * A function to generate a value to set the motor
 */
float motor_controller::update_motor(int deltaEncoder_ticks){
	
	static int prev_deltaEncoder_ticks = 0;	
	
	//check for the special condition
	if(deltaEncoder_ticks == -1 && prev_deltaEncoder_ticks == -1)
	{
		deltaEncoder_ticks = 0;
	}

	float setMotor;

	// calculate the position
	float deltaPosition_m = (((float)deltaEncoder_ticks) / this->ticks_per_rev) * (2 * M_PI * this->wheel_radius_m); 
	this->cur_pos_m += deltaPosition_m;
	prev_deltaEncoder_ticks = deltaEncoder_ticks;
	
	// calculate the velocity
	this->cur_vel_mps = deltaPosition_m/this->time_step_s;    

	// calculate PID value for the appropriate mode
	if(this->mode == 1) //VELOCITY
	{
	  //calcuate PID output
	  setMotor = this->vel_PID.getValue(this->cur_vel_mps,this->des_vel_mps,this->time_step_s); 
	  
	} else if(this->mode == 0) //POSITION
	{
	  //calcuate PID output
	  setMotor = this->pos_PID.getValue(this->cur_pos_m,this->des_pos_m,this->time_step_s);  
	}
	
	return setMotor;
}

/**
 * A function to publish the current state of the controller
 */
void motor_controller::update_feedback(){
	
	//publish the joint state
	sensor_msgs::JointState joint_msg;
	joint_msg.header.stamp = ros::Time::now();
	joint_msg.name.push_back(this->node_name);
	joint_msg.position.push_back(this->cur_pos_m);
	joint_msg.velocity.push_back(this->cur_vel_mps);
	joint_pub.publish(joint_msg);
	

	//publish all of the feedback
	oddbot_msgs::MotorFeedback fdbk_msg;
	fdbk_msg.header.stamp = ros::Time::now();
	fdbk_msg.item.push_back("cur_pos_m");
	fdbk_msg.value.push_back(this->cur_pos_m);
	fdbk_msg.item.push_back("cur_vel_mps");
        fdbk_msg.value.push_back(this->cur_vel_mps);
	fdbk_msg.item.push_back("wheel_radius_m");
        fdbk_msg.value.push_back(this->wheel_radius_m);
	fdbk_msg.item.push_back("max_vel_mps);
	fdbk_msg.value.push_back(this->max_vel_mps);
	fdbk_msg.item.push_back("cur_cur_amp");
        fdbk_msg.value.push_back(this->cur_cur_amp);
	fdbk_msg.item.push_back("time_step_s");
        fdbk_msg.value.push_back(this->time_step_s);
	fdbk_msg.item.push_back("mode");
        fdbk_msg.value.push_back(this->mode);
	fdbk_msg.item.push_back("timeout");
        fdbk_msg.value.push_back(this->timeout);
	fdbk_msg.item.push_back("des_pos_m");
        fdbk_msg.value.push_back(this->des_pos_m);
	fdbk_msg.item.push_back("cur_vel_mps");
        fdbk_msg.value.push_back(this->cur_vel_mps);
	fdbk_msg.item.push_back("ticks_per_rev");
        fdbk_msg.value.push_back(this->ticks_per_rev);
	feedback_pub.publish(fdbk_msg);
}

/**
 * This initializes the class laser_scanner_test and loops checking for new messages
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_controller");

	motor_controller mc = motor_controller();

	ROS_INFO("laser scanner test node started!");	
	
	// Initialize GPIO
	BlackLib::BlackGPIO ENA(BlackLib::GPIO_39,BlackLib::output, BlackLib::SecureMode);   
	BlackLib::BlackGPIO ENB(BlackLib::GPIO_38,BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO INA(BlackLib::GPIO_35,BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO INB(BlackLib::GPIO_34,BlackLib::output, BlackLib::SecureMode);

	//NEEDS to BE ANALOG VALUE
	//BlackLib::BlackGPIO   CS(BlackLib::GPIO_51,BlackLib::input, BlackLib::SecureMode);
	
	// INA - Motor direction input A (CW)
	// INB - Motor direction input B (CCW)
	// ENA - enable half bridge A (HIGH enabled)
	// ENB - enable half bridge B (HIGH enabled)
	// CS  -  Current Sense output
	
	// INA---INB---ENA---ENB---OUTA---OUTB---Operating Mode
	//  1     1     1     1     H       H     Brake to Vcc
	//  1     0     1     1     H       L     CW
	//  0     1     1     1     L       H     CCW
	//  0     0     1     1     L       L     Brake to GND
	
	ENA.setValue(BlackLib::high);          // enable half bridge A
	ENB.setValue(BlackLib::high);          // enable half bridge B
	INA.setValue(BlackLib::low);          // Turn on to set into brake 
	INB.setValue(BlackLib::low);          // turn on to set to Brake 
	
	//Initialize PWM
	BlackLib::BlackPWM pwmMotor(BlackLib::EHRPWM2A);
	pwmMotor.setDutyPercent(0.0); 
    pwmMotor.setPeriodTime(0.001 * pow(10,12), BlackLib::picosecond); 
    
	// Allocate an instane of eqep
	eQEP eqep(mc.getEqepPath(), eQEP::eQEP_Mode_Relative);

	// Set the unit time period on eqep
	eqep.set_period(mc.getTimeStepS() * pow(10,9));
	
	//initialize other params
	int deltaEncoder_ticks;
	float setMotor;

	while (ros::ok())
	{
		// check for updates
		ros::spinOnce();
		
		//get the change in encoder ticks
		deltaEncoder_ticks = eqep.get_position();
		
		//get the value to set the motor at
		setMotor = mc.update_motor(deltaEncoder_ticks);
		
		//set the Motor Speed
		if(setMotor < 0)
		{
			INA.setValue(BlackLib::high);
			INB.setValue(BlackLib::low);         
			setMotor = -1*setMotor;
		} else if(setMotor > 0) 
		{
			INA.setValue(BlackLib::low);
			INB.setValue(BlackLib::high);          
		} else if(setMotor == 0){
			INA.setValue(BlackLib::low);
			INB.setValue(BlackLib::low);          
		}

		//set the motor PWM value
		pwmMotor.setDutyPercent(setMotor);
		
		//get the current drawn
		//set  cur_cur_amp = CS.getValue();
		
		//Publish the update
		mc.update_feedback();
	}

	// turn off the motor;
	INA.setValue(BlackLib::low);
	INB.setValue(BlackLib::low); 
	pwmMotor.setDutyPercent(0.0);

	return 0;
}


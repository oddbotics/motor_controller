/*
 * \motor_controller.cpp
 * \controls the motor based on PID
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date February 14, 2015
 */

#include "motor_controller/motor_controller.hpp"

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
	private_node_handle_.param<double>("rate_s", time_step_s, 1.0);
	private_node_handle_.param<double>("pwm_period_s", period_s, 1.0);
	private_node_handle_.param<int>("mode", mode, 1);
	private_node_handle_.param<int>("ticks_per_rev", ticks_per_rev, 3200);
	private_node_handle_.param<double>("wheel_radius_m", wheel_radius_m, 0.0619125);
	private_node_handle_.param<double>("min_output", min_output, -100.0);
	private_node_handle_.param<double>("max_output", max_output, -100.0);
	
	// Initialize GPIO
	BlackLib::BlackGPIO ENA(BlackLib::GPIO_70,BlackLib::output, BlackLib::SecureMode);   
	BlackLib::BlackGPIO ENB(BlackLib::GPIO_71,BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO INA(BlackLib::GPIO_72,BlackLib::output, BlackLib::SecureMode);
	BlackLib::BlackGPIO INB(BlackLib::GPIO_73,BlackLib::output, BlackLib::SecureMode);

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
	
	ENA.setValue(BlackLib::high);          // turn on the led1
	ENB.setValue(BlackLib::high);          // turn on the led1
	INA.setValue(BlackLib::low);          // turn on the led1
	INB.setValue(BlackLib::low);          // turn on the led1
	
	//Initialize PWM
	BlackLib::BlackPWM pwmMotor(BlackLib::EHRPWM2A);
	pwmMotor.setDutyPercent(0.0);
	period_ps = period_s * pow(10,12);
    pwmMotor.setPeriodTime(period_ps, BlackLib::picosecond); //0.001 
    
	// Allocate an instane of eqep
	eQEP eqep(argv[1], eQEP::eQEP_Mode_Relative);

	// Set the unit time period on eqep
	time_step_ns = time_step_s * pow(10,9); 
	eqep.set_period(time_step_ns);

	// initialize PID loops
	vel_PID = PID(Kp_vel, Ki_vel, Kd_vel, min_vel_output, max_vel_output);
	pos_PID = PID(Kp_pos, Ki_pos, Kd_pos, min_pos_output, max_pos_output);
  
	//initialize the publishers and subscribers
	std::string node_name = ros::this_node::getName();
	feedback_pub = nh.advertise<sensor_msgs::JointState>(node_name,"/feedback"), 1000);
	control_sub = nh.subscribe(strcat(node_name,"/command"), 1000, &laser_scanner_test::update_map, this);
  
}

/**
 * a call back function that takes in the desired values for the 
 * controller and updates the variables accordingly
 */
void motor_controller::update_controller(){
	mode = 
	timeout =
	// desired control 
	float desVel_mps = 0.0;
	float desPos_m = 0.0;
}

void motor_controller::update_motor(){
	
	float setMotor = 0;
	static int prev_deltaEncoder_ticks = 0;
	static float prev_setMotor = 0;	
	// get the encoder count
	int deltaEncoder_ticks = eqep.get_position();
	//check for the special condition
	if(deltaEncoder_ticks == -1 && prev_deltaEncoder_ticks == -1)
	{
		deltaEncoder_ticks = 0;
	}

	// calculate the position
	float deltaPosition_m = (((float)deltaEncoder_ticks) / ticks_per_rev) * (2 * M_PI * wheel_radius_m); 
	curPos_m += deltaPosition_m;
	prev_deltaEncoder_ticks = deltaEncoder_ticks;
	
	// calculate the velocity
	float curVel_mps = deltaPosition_m/time_step_s;
	std::cout << desVel_mps << "," <<  curVel_mps << std::endl;     

	// calculate PID value for the appropriate mode
	if(mode == VELOCITY)
	{
	  //calcuate PID output
	  setMotor = vel_PID.getValue(curVel_mps,desVel_mps,time_step_s); 
	  
	} else if(mode == POSITION)
	{
	  //calcuate PID output
	  setMotor = pos_PID.getValue(curPos_m,desPos_m,time_step_s);  
	}
	
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
	prev_setMotor = setMotor;
	
	//get the current drawn
	//curCur_amp = CS.getValue();
	
	//Publish the update
	feedback_pub
}


/**
 * This initializes the class laser_scanner_test and loops checking for new messages
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_controller");

	motor_controller mc = motor_controller();

	ROS_INFO("laser scanner test node started!");	
	
	//initialize the BBB GPIO
	

	while (ros::ok())
	{
		mc.update_motor(pwm,ina,inb);
		ros::spinOnce();
	}

	// turn off the motor;
	INA.setValue(BlackLib::low);
	INB.setValue(BlackLib::low); 
	pwmMotor.setDutyPercent(0.0);

	return 0;
}

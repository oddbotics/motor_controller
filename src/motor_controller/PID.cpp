#include "motor_controller/PID.hpp"

PID::PID()
{
  this->Kp = 0.0;
  this->Ki = 0.0;
  this->Kd = 0.0;
  this->min_output = 0.0;
  this->max_output = 100.0;
}

PID::PID(double Kp, double Ki, double Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->min_output = 0.0;
  this->max_output = 100.0;
}

PID::PID(double Kp, double Ki, double Kd, double min_output, double max_output)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->min_output = min_output;
  this->max_output = max_output;
}

double PID::getValue(double curValue, double desValue, double time_step)
{
  static double sumError = 0;
  static double prevError = 0;

  double error = desValue - curValue;

  // calc potential term
  double potentialTerm = Kp * error;  

  // calc integral term
  sumError += error;
  double integralTerm = Ki *  sumError; 
  
  integralTerm = this->bound(integralTerm);

  // calc differential term
  double diffError = (error - prevError) / time_step;
  double differentialTerm = Kd * diffError;

  // calculate the PID value
  double pidValue = potentialTerm + integralTerm + differentialTerm;

  pidValue = this->bound(pidValue);

  return pidValue;   
}

double PID::bound(double value)
{
	//bound the output
  if(value > this->max_output)
  {
    value = this->max_output;
  } else if(value < this->min_output)
  {
    value = this->min_output;     
  }
  
  return value;
  
}





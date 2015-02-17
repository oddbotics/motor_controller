
#ifndef PID_HPP_
#define PID_HPP_

class PID
{
  public:
    PID();
    PID(double Kp, double Ki, double Kd);
    PID(double Kp, double Ki, double Kd, double min_output, double max_output);
    double getValue(double curValue, double prevValue, double time_step);

  private:
    double Kp;
    double Ki;
    double Kd;
    double time_step; //seconds
    double min_output;
    double max_output;
    double bound(double value);
};

#endif

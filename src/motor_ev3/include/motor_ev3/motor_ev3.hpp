#ifndef MOTOR_EV3_HPP
#define MOTOR_EV3_HPP


class MotorEv3
{
public:
    void init();
    void reset();
  

private:
  double degrees = 0.0;
  double rotations = 0.0;
  float power = 0.0;


};

#endif
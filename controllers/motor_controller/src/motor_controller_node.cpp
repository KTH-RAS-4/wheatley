
#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <math.h>

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

class Motor
{
public:
    double kP;
    double kI;
    double kD;
    double I;
    double last_error;

    Motor()
        : kP(0), kI(0), kD(0), I(0)
        , last_error(0)
    {
    }

    int update(double elapsed, double error)
    {
        double D = (error - last_error) / elapsed;
        I += error*elapsed;
        last_error = error;
        return (int) (kP*error + kD*D + kI*I);
    }
};

class MotorControllerNode
{
public:

  ros::NodeHandle n;
  ros::Publisher pwmPub;
  ros::Publisher desiredAngularVelocityLeftPub;
  ros::Publisher desiredAngularVelocityRightPub;
  ros::Publisher measuredAngularVelocityLeftPub;
  ros::Publisher measuredAngularVelocityRightPub;

  ros::Subscriber encode;
  ros::Subscriber twist;

  
  MotorControllerNode(double rate)
      : loop_rate(rate)
  {
    n = ros::NodeHandle("~");
    pwm.PWM1 = 0;
    pwm.PWM2 = 0;
    init();
  }
  
  ~MotorControllerNode()
  {
  }
  
  void init()
  {
    pwmPub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    desiredAngularVelocityLeftPub   = n.advertise<std_msgs::Float64>("/motors/left/desiredAngular",   1);
    desiredAngularVelocityRightPub  = n.advertise<std_msgs::Float64>("/motors/right/desiredAngular",  1);
    measuredAngularVelocityLeftPub  = n.advertise<std_msgs::Float64>("/motors/left/measuredAngular",  1);
    measuredAngularVelocityRightPub = n.advertise<std_msgs::Float64>("/motors/right/measuredAngular", 1);

    encode = n.subscribe("/arduino/encoders", 1000, &MotorControllerNode::encCallback, this);
    twist = n.subscribe("/motor_controller/twist", 1000, &MotorControllerNode::twistCallback, this);
  }
  void encCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
  {
    enc.delta_encoder1 = -msg->delta_encoder1;
    enc.delta_encoder2 = -msg->delta_encoder2;
  }
  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
  { 
    twi.linear.x = msg->linear.x;
    twi.angular.z = msg->angular.z;
  }

  void run()
  {
      while(ros::ok()){
        ros::spinOnce();
        calc();
        loop_rate.sleep();
      }
  }



  void calc()
  {
    n.getParam("/settings/pid/left/P", motorL.kP);
    n.getParam("/settings/pid/left/I", motorL.kI);
    n.getParam("/settings/pid/left/D", motorL.kD);
    n.getParam("/settings/pid/right/P", motorR.kP);
    n.getParam("/settings/pid/right/I", motorR.kI);
    n.getParam("/settings/pid/right/D", motorR.kD);

    double elapsed = loop_rate.expectedCycleTime().toSec();
    double wDesiredL = (twi.linear.x-(0.5*0.238*twi.angular.z))/(0.0975/2);
    double wDesiredR = (twi.linear.x+(0.5*0.238*twi.angular.z))/(0.0975/2);
    double wMeasuredL = ((double) (enc.delta_encoder1)*2*M_PI*10)/360;
    double wMeasuredR = ((double) (enc.delta_encoder2)*2*M_PI*10)/360;
    double errorL = wDesiredL - wMeasuredL;
    double errorR = wDesiredR - wMeasuredR;

    pwm.PWM1 =   motorL.update(elapsed, errorL);
    pwm.PWM2 = - motorR.update(elapsed, errorR);

    pwm.PWM1 = CLAMP(pwm.PWM1, -255, 255);
    pwm.PWM2 = CLAMP(pwm.PWM2, -255, 255);

    /*ROS_INFO("P: %.2f", motorR.P);
    ROS_INFO("I: %.2f", motorR.I);
    ROS_INFO("D: %.2f", motorR.D);*/

    /*ras_arduino_msgs::PWM pwm_msg;
    pwm_msg = pwm;
    if(pwm_msg.PWM1 < 30) pwm_msg.PWM1 = 0;
    if(pwm_msg.PWM2 > -30) pwm_msg.PWM2 = 0;*/

    pwmPub.publish(pwm);
    std_msgs::Float64 dw1m; dw1m.data = wDesiredL;
    std_msgs::Float64 dw2m; dw2m.data = wDesiredR;
    std_msgs::Float64 mw1m; mw1m.data = wMeasuredL;
    std_msgs::Float64 mw2m; mw2m.data = wMeasuredR;
    desiredAngularVelocityLeftPub.publish(dw1m);
    desiredAngularVelocityRightPub.publish(dw2m);
    measuredAngularVelocityLeftPub.publish(mw1m);
    measuredAngularVelocityRightPub.publish(mw2m);

    ROS_INFO(" desired: [%6.1f, %6.1f]", wDesiredL, wDesiredR);
    ROS_INFO("  actual: [%6.1f, %6.1f]", wMeasuredL, wMeasuredR);
    ROS_INFO("   error: [%6.1f, %6.1f]", errorL, errorR);
    ROS_INFO("integral: [%6.1f, %6.1f]", motorL.I, motorR.I);
    ROS_INFO("     PWM: [%6d, %6d]", pwm.PWM1, pwm.PWM2);
  }
  
private:
  Motor motorL;
  Motor motorR;

  ras_arduino_msgs::PWM pwm;
  ras_arduino_msgs::Encoders enc;
  geometry_msgs::Twist twi;
  ros::Rate loop_rate;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller_node");
  MotorControllerNode my_node = MotorControllerNode(10.0);
  my_node.run();
  return 0;
}

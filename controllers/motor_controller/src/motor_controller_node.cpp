
#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <boost/circular_buffer.hpp>

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

class Motor
{
private:
    ros::NodeHandle n;
public:
    //gains
    double kP;
    double kI;
    double kD;
    //transform
    double constant;
    double exponent;
    double min;
    int max;
    //other
    double I;
    int I_memory;
    double last_error;
    boost::circular_buffer<double> errors;

public:
    Motor(std::string paramPath)
        : n(paramPath)
        , kP(0), kI(0), kD(0)
        , constant(0), exponent(0)
        , min(0), max(0)
        , I(0), I_memory(0)
        , last_error(0)
    {
        if (!n.hasParam("gains"))
        {
            ROS_FATAL("no parameter %s/gains found", paramPath.data());
            ros::shutdown();
        }

        n.getParam("gains/P", kP);
        n.getParam("gains/I", kI);
        n.getParam("gains/D", kD);
        n.getParam("transform/constant", constant);
        n.getParam("transform/exponent", exponent);
        n.getParam("transform/min", min);
        n.getParam("transform/max", max);
        n.getParam("gains/I_memory", I_memory);
        errors = boost::circular_buffer<double>(I_memory, I_memory, 0); //pre-filled buffer
        //ROS_INFO("listening on %s", n.getNamespace().data());
    }

    int update(double elapsed, double error)
    {
        double D = (error - last_error) / elapsed;
        last_error = error;

        if (I_memory > 0)
            I -= errors.front();
        I += error*elapsed;
        errors.push_back(error*elapsed);

        double speed = kP*error + kD*D + kI*I;
        int sign = sgn(speed);
        speed = fabs(speed);

        int pwm = (int) (pow(constant*speed, exponent)+min);
        ROS_INFO("pwm:%d speed:%f", pwm, speed);

        pwm = CLAMP(pwm, 0, max);
        return sign*pwm; //if speed==0, sign==0
    }
};

class MotorControllerNode
{
private:
  ros::NodeHandle n;
  ros::Publisher pwmPub;
  ros::Publisher desiredAngularVelocityLeftPub;
  ros::Publisher desiredAngularVelocityRightPub;
  ros::Publisher measuredAngularVelocityLeftPub;
  ros::Publisher measuredAngularVelocityRightPub;

  ros::Subscriber encode;
  ros::Subscriber twist;

  Motor motorL;
  Motor motorR;

  ras_arduino_msgs::PWM pwm;
  ras_arduino_msgs::Encoders enc;
  geometry_msgs::Twist twi;
  ros::Rate loop_rate;
  ros::Time last_input;

  double timeout;

public:
  MotorControllerNode()
      : n("~")
      , motorL("~/motors")
      , motorR("~/motors")
      , last_input(-1000)
      , loop_rate(1)
  {
    //ROS_INFO("listening on %s", n.getNamespace());
    pwm.PWM1 = 0;
    pwm.PWM2 = 0;
    init();
  }

  ~MotorControllerNode()
  {
  }

  void init()
  {
    double rate;
    n.getParam("rate", rate);
    loop_rate = ros::Rate(rate);

    n.getParam("timeout", timeout);

    pwmPub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    desiredAngularVelocityLeftPub   = n.advertise<std_msgs::Float64>("motors/left/desiredAngular",   1);
    desiredAngularVelocityRightPub  = n.advertise<std_msgs::Float64>("motors/right/desiredAngular",  1);
    measuredAngularVelocityLeftPub  = n.advertise<std_msgs::Float64>("motors/left/measuredAngular",  1);
    measuredAngularVelocityRightPub = n.advertise<std_msgs::Float64>("motors/right/measuredAngular", 1);

    encode = n.subscribe("/arduino/encoders", 1000, &MotorControllerNode::encCallback, this);
    twist = n.subscribe("/motor_controller/twist", 1000, &MotorControllerNode::twistCallback, this);
  }
  void encCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
  {
    //TODO: += to integrate encoder deltas, and then reset them when read
    enc.delta_encoder1 = -msg->delta_encoder1;
    enc.delta_encoder2 = -msg->delta_encoder2;
  }
  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    twi.linear.x = msg->linear.x;
    twi.angular.z = msg->angular.z;
    last_input = ros::Time::now();
  }

  void run()
  {
      while(ros::ok())
      {
        ros::spinOnce();
        calc();
        loop_rate.sleep();
      }
  }



  void calc()
  {
    double elapsed = loop_rate.expectedCycleTime().toSec();
    double wDesiredL = (twi.linear.x-(0.5*0.238*twi.angular.z))/(0.0975/2);
    double wDesiredR = (twi.linear.x+(0.5*0.238*twi.angular.z))/(0.0975/2);
    double wMeasuredL = ((double) (enc.delta_encoder1)*2*M_PI*10)/360;
    double wMeasuredR = ((double) (enc.delta_encoder2)*2*M_PI*10)/360;

    bool timedOut = (ros::Time::now()-last_input).toSec() > timeout;

    if (timedOut)
        wDesiredL = wDesiredR = 0;

    double errorL = wDesiredL - wMeasuredL;
    double errorR = wDesiredR - wMeasuredR;

    if (timedOut)
    {
        pwm.PWM1 = 0;
        pwm.PWM2 = 0;
        motorL.I = 0;
        motorR.I = 0;
    }
    else
    {
        pwm.PWM1 =   motorL.update(elapsed, errorL);
        pwm.PWM2 = - motorR.update(elapsed, errorR);
    }

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
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  MotorControllerNode my_node;
  my_node.run();
  return 0;
}

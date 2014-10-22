
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
    double cutoff;
    int max;
    //other
    double I;
    double last_error;

public:
    Motor(std::string paramPath)
        : n(paramPath)
        , kP(0), kI(0), kD(0)
        , constant(0), exponent(0)
        , cutoff(0), max(0)
        , I(0), last_error(0)
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
        n.getParam("transform/min", cutoff);
        n.getParam("transform/max", max);
        ROS_INFO("listening on %s", n.getNamespace().data());
    }

    int update(double elapsed, double error)
    {
        double D = (error - last_error) / elapsed;
        I += error*elapsed;
        last_error = error;
        double speed = kP*error + kD*D + kI*I;

        int pwm;
        if (fabs(speed) < cutoff)
            pwm = 0;
        else if (speed > 0)
            pwm = (int)(constant*pow(speed-cutoff,exponent));
        else
            pwm = (int)(constant*pow(speed+cutoff,exponent));

        pwm = CLAMP(pwm, -max, max);
        return pwm;
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

private:
  double timeout;
public:
  MotorControllerNode(double rate)
      : loop_rate(rate)
      , n("~")
      , motorL("~/motors")
      , motorR("~/motors")
      , last_input(-1000)
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
      while(ros::ok()){
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
  MotorControllerNode my_node = MotorControllerNode(10.0);
  my_node.run();
  return 0;
}

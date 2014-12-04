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

geometry_msgs::Twist twi;

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
        errors = boost::circular_buffer<double>(I_memory, 0); //pre-filled buffer


        //ROS_INFO("listening on %s", n.getNamespace().data());
    }

    int update(double elapsed, double error)
    {
        double D = (error - last_error) / elapsed;
        last_error = error;
        double speed = 0;
        if (twi.linear.x != 0)
        {
            if (I_memory > 0)
            {
                I -= errors.front();
            }
            I += error*elapsed;
            errors.push_back(error*elapsed);

            speed = kP*error + kD*D + kI*I;
        } else
        {
            speed = kP*error + kD*D;
            errors = boost::circular_buffer<double>(I_memory, 0);

        }
        int sign = sgn(speed);

        int pwm = (int) (pow(constant*fabs(speed), exponent)+min);
        //ROS_INFO("pwm:%d speed:%f", pwm, speed);

        pwm = CLAMP(pwm, 0, max);
        return sign*pwm; //if speed==0, sign==0
    }
};

class MotorControllerNode
{
private:
  ros::NodeHandle n;
  ros::Publisher pub_pwm;
  ros::Publisher pub_left_desiredAngular;
  ros::Publisher pub_right_desiredAngular;
  ros::Publisher pub_left_measuredAngular;
  ros::Publisher pub_right_measuredAngular;

  ros::Subscriber sub_encode;
  ros::Subscriber sub_twist;

  Motor motorL;
  Motor motorR;

  ras_arduino_msgs::Encoders enc;

  ros::Rate loop_rate;
  ros::Time last_input;

  double timeout;
  bool output_measured_angular;

public:
  MotorControllerNode()
      : n("~")
      , motorL("~/motors")
      , motorR("~/motors")
      , last_input(-1000)
      , loop_rate(1)
      , output_measured_angular(false)
  {
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
    n.getParam("output_measured_angular", output_measured_angular);

    if (output_measured_angular)
    {
        pub_left_desiredAngular   = n.advertise<std_msgs::Float64>("motors/left/desiredAngular",   1);
        pub_right_desiredAngular  = n.advertise<std_msgs::Float64>("motors/right/desiredAngular",  1);
        pub_left_measuredAngular  = n.advertise<std_msgs::Float64>("motors/left/measuredAngular",  1);
        pub_right_measuredAngular = n.advertise<std_msgs::Float64>("motors/right/measuredAngular", 1);
    }

    pub_pwm = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);

    sub_encode = n.subscribe("/arduino/encoders", 1, &MotorControllerNode::encoderCallback, this);
    sub_twist = n.subscribe("twist", 1, &MotorControllerNode::twistCallback, this);
  }

  void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
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
    static bool timedOut = true;

    if ((ros::Time::now()-last_input).toSec() > timeout)
    {
        if (!timedOut)
        {
            timedOut = true;
            ROS_INFO("timeout");
            motorL.I = 0;
            motorR.I = 0;
        }
        ras_arduino_msgs::PWM pwm;
        pub_pwm.publish(pwm);
    }
    else
    {
        timedOut = false;
        double elapsed = loop_rate.expectedCycleTime().toSec();
        double wDesiredL = (twi.linear.x-(0.5*0.238*twi.angular.z))/(0.0975/2);
        double wDesiredR = (twi.linear.x+(0.5*0.238*twi.angular.z))/(0.0975/2);
        double wMeasuredL = ((double) (enc.delta_encoder1)*2*M_PI*10)/360;
        double wMeasuredR = ((double) (enc.delta_encoder2)*2*M_PI*10)/360;

        double errorL = wDesiredL - wMeasuredL;
        double errorR = wDesiredR - wMeasuredR;

        ras_arduino_msgs::PWM pwm;
        pwm.PWM1 =   motorL.update(elapsed, errorL);
        pwm.PWM2 = - motorR.update(elapsed, errorR);
        pub_pwm.publish(pwm);

        if (output_measured_angular)
        {
            std_msgs::Float64 ld; ld.data = wDesiredL;
            std_msgs::Float64 rd; rd.data = wDesiredR;
            std_msgs::Float64 lm; lm.data = wMeasuredL;
            std_msgs::Float64 rm; rm.data = wMeasuredR;

            pub_left_desiredAngular.publish(ld);
            pub_right_desiredAngular.publish(rd);
            pub_left_measuredAngular.publish(lm);
            pub_right_measuredAngular.publish(rm);
        }
        /*ROS_INFO(" desired: [%6.1f, %6.1f]", wDesiredL, wDesiredR);
        ROS_INFO("  actual: [%6.1f, %6.1f]", wMeasuredL, wMeasuredR);
        ROS_INFO("   error: [%6.1f, %6.1f]", errorL, errorR);
        ROS_INFO("integral: [%6.1f, %6.1f]", motorL.I, motorR.I);*/
        //ROS_INFO("     PWM: [%6d, %6d]", pwm.PWM1, pwm.PWM2);

    }
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  MotorControllerNode my_node;
  my_node.run();
}

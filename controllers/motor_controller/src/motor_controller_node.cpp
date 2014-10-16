
#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class MotorControllerNode
{
  
public:

  ros::NodeHandle n;
  ros::Publisher pub;
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
    pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    encode = n.subscribe("/arduino/encoders", 1000, &MotorControllerNode::encCallback, this);
    twist = n.subscribe("/motor_controller/twist", 1000, &MotorControllerNode::twistCallback, this);
    last_error1 = 0;
    last_error2 = 0;
    I1 = 0;
    I2 = 0;
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
    n.getParam("/settings/pid/Kp1", Kp1);
    n.getParam("/settings/pid/Ki1", Ki1);
    n.getParam("/settings/pid/Kd1", Kd1);
    n.getParam("/settings/pid/Kp2", Kp2);
    n.getParam("/settings/pid/Ki2", Ki2);
    n.getParam("/settings/pid/Kd2", Kd2);

    pwm.PWM1 = 0;
    pwm.PWM2 = 0;

    double elapsed = loop_rate.expectedCycleTime().toSec();

    double desired_w1 = (twi.linear.x-(0.5*0.238*twi.angular.z))/(0.0975/2);
    double desired_w2 = (twi.linear.x+(0.5*0.238*twi.angular.z))/(0.0975/2);
    double estimated_w1 = ((double) (enc.delta_encoder1)*2*M_PI*10)/360;
    double estimated_w2 = ((double) (enc.delta_encoder2)*2*M_PI*10)/360;
    double error1 = desired_w1 - estimated_w1;
    double error2 = desired_w2 - estimated_w2;
    double D1 = (error1 - last_error1)/ elapsed;
    double D2 = (error2 - last_error2)/ elapsed;
    double I1 = I1 + Ki1*(error1*elapsed);
    double I2 = I2 + Ki2*(error2*elapsed);
    pwm.PWM1 = (int)(Kp1*error1) + (int) (Kd1*D1) + (int) I1;
    pwm.PWM2 = -((int)(Kp2*error2) + (int) (Kd2*D2) + (int) I2);
    last_error1 = error1;
    last_error2 = error2;

    ROS_INFO("%.2f", Kp1);
    ROS_INFO("%.2f", Ki1);
    ROS_INFO("%.2f", Kd1);

    ras_arduino_msgs::PWM pwm_msg;
    pwm_msg = pwm;
    if(pwm_msg.PWM1 < 30) pwm_msg.PWM1 = 0;
    if(pwm_msg.PWM2 > -30) pwm_msg.PWM2 = 0;
    pub.publish(pwm);
    ROS_INFO("[%.1f, %.1f] estimated: [%.1f, %.1f], PWM: [%d, %d]",desired_w1, desired_w2, estimated_w1, estimated_w2, pwm.PWM1, pwm.PWM2);
  }
  
private:
  double last_error1;
  double last_error2;
  double I1;
  double I2;
  double Kp1;
  double Ki1;
  double Kd1;
  double Kp2;
  double Ki2;
  double Kd2;
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

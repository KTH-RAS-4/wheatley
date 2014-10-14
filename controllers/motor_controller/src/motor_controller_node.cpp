
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
  
  MotorControllerNode()
  {
    n = ros::NodeHandle("~");
    pwm.PWM1 = 0;
    pwm.PWM2 = 0;
  }
  
  ~MotorControllerNode()
  {
  }
  
  void init()
  {
    pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    encode = n.subscribe("/arduino/encoders", 1000, &MotorControllerNode::encCallback, this);
    twist = n.subscribe("/motor_controller/twist", 1000, &MotorControllerNode::twistCallback, this);
  }
  void encCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
  {
    enc.delta_encoder1 = -msg->delta_encoder1;
    enc.delta_encoder2 = msg->delta_encoder2;
  }
  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
  { 
    twi.linear.x = msg->linear.x;
    twi.angular.z = msg->angular.z;
  }
  
  void calc()
  {
    double desired_w1 = (twi.linear.x-(0.5*0.238*twi.angular.z))/0.0975;
    double desired_w2 = (twi.linear.x+(0.5*0.238*twi.angular.z))/0.0975;
    double estimated_w1 = ((double) (enc.delta_encoder1)*2*M_PI*10)/360;
    double estimated_w2 = ((double) (enc.delta_encoder2)*2*M_PI*10)/360;
    pwm.PWM1 = pwm.PWM1 + (int)(2*(desired_w1 - estimated_w1));
    pwm.PWM2 = pwm.PWM2 + (int)(2*(desired_w2 - estimated_w2));
    pub.publish(pwm);
    ROS_INFO("[%f, %f] estimated: [%f, %f]",desired_w1, desired_w2, estimated_w1, estimated_w2);
  }
  
private:
  ras_arduino_msgs::PWM pwm;
  ras_arduino_msgs::Encoders enc;
  geometry_msgs::Twist twi;
  
};
int main (int argc, char **argv){
  ros::init(argc, argv, "motor_controller_node");
  MotorControllerNode my_node = MotorControllerNode();
  my_node.init();
  ros::Rate loop_rate(10.0);
  while(ros::ok()){
    ros::spinOnce();
    my_node.calc();
    loop_rate.sleep();
  }
  return 0;
}

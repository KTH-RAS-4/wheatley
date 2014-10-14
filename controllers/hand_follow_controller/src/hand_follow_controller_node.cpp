
#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main (int argc, char **argv){
  ros::init(argc, argv, "open_loop_controller");
  ros::NodeHandle n;
  double v = 0.2;
  double w = 0;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
  ros::Rate loop_rate(10.0);
  geometry_msgs::Twist twi;
  twi.linear.x = v;
  twi.angular.z = w;
  while(ros::ok()){
    pub.publish(twi);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

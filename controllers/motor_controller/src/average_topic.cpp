#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <std_msgs/Float64.h>

ras_arduino_msgs::ADConverter average;
ras_arduino_msgs::ADConverter sum;

ros::Publisher pub;
float counter;

void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
{
  sum.ch1 += msg->ch1;
  sum.ch2 += msg->ch2;
  sum.ch3 += msg->ch3;
  sum.ch4 += msg->ch4;
  sum.ch5 += msg->ch5;
  sum.ch6 += msg->ch6;
  sum.ch7 += msg->ch7;
  sum.ch8 += msg->ch8;

  counter++;

  average.ch1 = sum.ch1 / counter;
  average.ch2 = sum.ch2 / counter;
  average.ch3 = sum.ch3 / counter;
  average.ch4 = sum.ch4 / counter;
  average.ch5 = sum.ch5 / counter;
  average.ch6 = sum.ch6 / counter;
  average.ch7 = sum.ch7 / counter;
  average.ch8 = sum.ch8 / counter;

  pub.publish(average);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sum_topic");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/arduino/adc", 1, adcCallback);

  counter = 0;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<ras_arduino_msgs::ADConverter> ("/adc/average", 1);

  ros::Rate loop_rate(10.0);

  while(ros::ok()){
    //ros::spin();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

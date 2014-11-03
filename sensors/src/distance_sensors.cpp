#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_arduino_msgs/Distance.h>
#include <math.h>

ras_arduino_msgs::Distance out;
ras_arduino_msgs::ADConverter in;

ros::Publisher pub;

float poly(int ch, float p1, float p2, float p3, float p4, float p5, float p6){
    return p1*pow(ch,5) + p2*pow(ch,4) + p3*pow(ch,3) + p4*pow(ch,2) + p5*ch + p6;
}

void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
{
  in.ch1 += msg->ch1;
  in.ch2 += msg->ch2;
  in.ch3 += msg->ch3;
  in.ch4 += msg->ch4;
  in.ch5 += msg->ch5;
  in.ch6 += msg->ch6;
  in.ch7 += msg->ch7;
  in.ch8 += msg->ch8;


  out.left_front = poly(in.ch1, -0.2579, 0.839, -0.9278, 1.722, -5.946, 7.166);
  out.front = poly(in.ch2, -0.5123, 3.039, -6.709, 9.331, -15.43, 17.61);
  out.left_rear = poly(in.ch3, -0.322, 1.018, -1.076, 2.008, -6.101, 6.938);
  out.right_rear = poly(in.ch4, 0.09064, -0.1877, -0.703, 2.854, -6.393, 7.013);
  out.rear = poly(in.ch5, -0.371, 2.583, -6.9, 10.51, -15.56, 17.27);
  out.right_front = poly(in.ch6, -0.3894, 1.293, -1.416, 1.936, -5.762, 6.9);


  pub.publish(out);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "in_topic");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/arduino/adc", 1, adcCallback);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<ras_arduino_msgs::Distance> ("/sensors/distance", 1);

  ros::Rate loop_rate(10.0);

  while(ros::ok()){
    //ros::spin();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <sensors/Distance.h>
#include <cmath>

ros::Publisher pub;

struct param_t {
  double p1;
  double p2;
  double p3;
  double p4;
  double p5;
  double p6;
} param_ch1, param_ch2, param_ch3, param_ch4, param_ch5, param_ch6;

void init() {
  param_ch1.p1 = -1.337e-10;
  param_ch1.p2 = 1.445e-07;
  param_ch1.p3 = -6.21e-05;
  param_ch1.p4 = 0.0135;
  param_ch1.p5 = -1.572;
  param_ch1.p6 = 87.45;

  param_ch2.p1 = -3.844e-11;
  param_ch2.p2 = 6.21e-08;
  param_ch2.p3 = -3.971e-05;
  param_ch2.p4 = 0.01279;
  param_ch2.p5 = -2.171;
  param_ch2.p6 = 170.5;

  param_ch3.p1 = -1.404e-10;
  param_ch3.p2 = 1.598e-07;
  param_ch3.p3 = -7.228e-05;
  param_ch3.p4 = 0.01651;
  param_ch3.p5 = -1.993;
  param_ch3.p6 = 111;

  param_ch4.p1 = 4.222e-11;
  param_ch4.p2 = -4.48e-08;
  param_ch4.p3 = 1.686e-05;
  param_ch4.p4 = -0.002318;
  param_ch4.p5 = -0.06936;
  param_ch4.p6 = 35.5;

  param_ch5.p1 = -3.839e-11;
  param_ch5.p2 = 6.468e-08;
  param_ch5.p3 = -4.329e-05;
  param_ch5.p4 = 0.01456;
  param_ch5.p5 = -2.545;
  param_ch5.p6 = 198.9;

  param_ch6.p1 = -2.565e-10;
  param_ch6.p2 = 2.735e-07;
  param_ch6.p3 = -0.0001143;
  param_ch6.p4 = 0.02371;
  param_ch6.p5 = -2.533;
  param_ch6.p6 = 122.1;
}

float poly(int ch, param_t params)
{
    return params.p1*pow(ch,5) + params.p2*pow(ch,4) + params.p3*pow(ch,3) + params.p4*pow(ch,2) + params.p5*ch + params.p6;
}

void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
{
  sensors::Distance out;
  out.header.stamp = ros::Time::now();
  out.left_front = round(poly(msg->ch1, param_ch1)*10)/1000;
  out.front = round(poly(msg->ch2, param_ch2)*10)/1000;
  out.left_rear = round(poly(msg->ch3, param_ch3)*10)/1000;
  out.right_rear = round(poly(msg->ch4, param_ch4)*10)/1000;
  out.rear = round(poly(msg->ch5, param_ch5)*10)/1000;
  out.right_front = round(poly(msg->ch6, param_ch6)*10)/1000;

  pub.publish(out);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "distance_sensors_node");
  ros::NodeHandle nh("~");

  init();
  pub = nh.advertise<sensors::Distance> ("", 100);
  ros::Subscriber sub = nh.subscribe ("/arduino/adc", 100, adcCallback);

  ros::spin();
}

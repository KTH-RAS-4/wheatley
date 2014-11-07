#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/Odometry.h>
#define _USE_MATH_DEFINES
#include <cmath>

ros::Publisher pub;
ras_arduino_msgs::Odometry out;

//put in as constants
float wheeldiameter = 0.1;
float basediameter = 0.21;
float countPerRevolution = 360;
float degreesPerCount = (wheeldiameter/basediameter) / 2;
float distancePerCount = M_PI * wheeldiameter / countPerRevolution;


void init()
{

}

void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
{
    out.theta += (msg->delta_encoder1 - msg->delta_encoder2)*degreesPerCount;
    float deltaDistance =(msg->delta_encoder1 +msg->delta_encoder2) / 2.0 *distancePerCount;
    out.x -= deltaDistance*std::sin(out.theta*M_PI/180);
    out.y -= deltaDistance*std::cos(out.theta*M_PI/180);

    pub.publish(out);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pose_node");
    ros::NodeHandle nh("~");

    pub = nh.advertise<ras_arduino_msgs::Odometry> ("", 1);
    ros::Subscriber sub = nh.subscribe ("/arduino/encoders", 1, encoderCallback);

    ros::spin();
}

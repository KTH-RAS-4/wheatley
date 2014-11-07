#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/Odometry.h>
#include <math.h>

ras_arduino_msgs::Odometry out;
ras_arduino_msgs::Encoders in;

ros::Publisher pub;

float x = 0;
float y = 0;
float theta = 0;
//put in as constants
float wheeldiameter = 0.1;
float basediameter = 0.21;
float countPerRevolution = 360;
float degreesPerCount = (wheeldiameter/basediameter) / 2;
float distancePerCount = 3.14159 * wheeldiameter / countPerRevolution;

void init(){

}


void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
{
    out.theta =  out.theta + (msg->delta_encoder1 - msg->delta_encoder2)*degreesPerCount;
    float deltaDistance =(msg->delta_encoder1 +msg->delta_encoder2) / 2.0 *distancePerCount;
    out.x = out.x - deltaDistance*std::sin(out.theta*3.14159/180);
    out.y = out.y - deltaDistance*std::cos(out.theta*3.14159/180);


    pub.publish(out);

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "in_topic");
    ros::NodeHandle nh;
    init();
    ros::Subscriber sub = nh.subscribe ("/arduino/encoders", 1, encoderCallback);


    pub = nh.advertise<ras_arduino_msgs::Odometry> ("/pose", 1);

    ros::Rate loop_rate(10.0);

    while(ros::ok()){
        //ros::spin();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

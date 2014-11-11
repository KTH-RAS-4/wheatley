#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#define _USE_MATH_DEFINES
#include <cmath>

ros::Publisher pub_pose;
ros::Publisher pub_pose_ros;
ras_arduino_msgs::Odometry pose;

//put in as constants
float wheeldiameter = 0.1;
float basediameter = 0.21;
float countPerRevolution = 360;
float degreesPerCount = (wheeldiameter/basediameter) / 2;
float distancePerCount = M_PI * wheeldiameter / countPerRevolution;



void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
{
    ros::Time now = ros::Time::now();

    //simple pose
    pose.theta += (msg->delta_encoder1 - msg->delta_encoder2)*degreesPerCount;
    float deltaDistance =(msg->delta_encoder1 +msg->delta_encoder2) / 2.0 *distancePerCount;
    pose.x -= deltaDistance*std::sin(pose.theta*M_PI/180);
    pose.y -= deltaDistance*std::cos(pose.theta*M_PI/180);
    pub_pose.publish(pose);

    //"robot" transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.x, pose.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, (90-pose.theta)*M_PI/180);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, now, "map", "robot"));

    //ros standard pose
    nav_msgs::Odometry pose_ros;
    pose_ros.header.stamp = now;
    pose_ros.header.frame_id = "robot";
    pose_ros.pose.pose.orientation.w = 1;
    pub_pose_ros.publish(pose_ros);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pose_node");
    ros::NodeHandle nh("~");

    pub_pose = nh.advertise<ras_arduino_msgs::Odometry> ("", 1);
    pub_pose_ros = nh.advertise<nav_msgs::Odometry> ("ros_pose", 1);
    ros::Subscriber sub = nh.subscribe ("/arduino/encoders", 1, encoderCallback);

    ros::spin();
}

#include <ros/ros.h>
#include <sensors/Distance.h>
#include <ras_arduino_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub;
ras_arduino_msgs::Odometry pose;
sensors::Distance distance;

void calc()
{
    visualization_msgs::Marker points;
    points.header.frame_id = "/map";

    points.header.stamp = ros::Time::now();
    points.ns = "map_points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    static int id = 0;
    points.id = id++;

    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    points.color.g = 1.0f;
    points.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = pose.x;
    p.y = pose.y;

    points.points.push_back(p);

    pub.publish(points);
}

void distanceCallback(const sensors::Distance::ConstPtr &msg)
{
    distance = *msg;
    calc();
}

void poseCallback(const ras_arduino_msgs::Odometry::ConstPtr &msg)
{
    pose = *msg;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "mapper");
    ros::NodeHandle nh("~");

    pub = nh.advertise<visualization_msgs::Marker>("", 10);
    ros::Subscriber sub_pose = nh.subscribe ("/sensors/pose", 1, poseCallback);
    ros::Subscriber sub_distance = nh.subscribe ("/sensors/distance", 1, distanceCallback);

    ros::spin();
}

#include <ros/ros.h>
#include <sensors/Distance.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

ros::Publisher pub_map;
sensors::Distance distance;

void calc()
{
    //TODO: really we would like to have gotten the time from somewhere else
    ros::Time now = ros::Time::now();

    visualization_msgs::Marker points;
    points.header.frame_id = "robot";

    points.header.stamp = now;
    points.ns = "map_points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    static int id = 0;
    points.id = id++;

    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.05;
    points.scale.y = 0.05;

    points.color.g = 1.0;
    points.color.a = 1.0;

    points.points.push_back(geometry_msgs::Point());

    pub_map.publish(points);
}

void distanceCallback(const sensors::Distance::ConstPtr &msg)
{
    distance = *msg;
    calc();
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "mapper");
    ros::NodeHandle nh("~");

    pub_map = nh.advertise<visualization_msgs::Marker>("", 10);
    ros::Subscriber sub_distance = nh.subscribe ("/sensors/distance", 1, distanceCallback);

    ros::spin();
}

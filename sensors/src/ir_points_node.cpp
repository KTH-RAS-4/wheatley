#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensors/Distance.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

class IrPointsNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_distance;
    ros::Publisher pub_ir;

    tf::StampedTransform tf_front;
    tf::StampedTransform tf_rear;
    tf::StampedTransform tf_left_front;
    tf::StampedTransform tf_left_rear;
    tf::StampedTransform tf_right_front;
    tf::StampedTransform tf_right_rear;

public:
    IrPointsNode()
        : nh("~")
    {
        tf::TransformListener tfl;
        ros::Time now(0); //get latest

        while (!tfl.waitForTransform("ir_front", "robot", now, ros::Duration(1)))
            ROS_ERROR("Couldn't find transform from 'robot' to 'ir_front', retrying...");

        tfl.lookupTransform("robot", "ir_front",       now, tf_front);
        tfl.lookupTransform("robot", "ir_rear",        now, tf_rear);
        tfl.lookupTransform("robot", "ir_left_front",  now, tf_left_front);
        tfl.lookupTransform("robot", "ir_left_rear",   now, tf_left_rear);
        tfl.lookupTransform("robot", "ir_right_front", now, tf_right_front);
        tfl.lookupTransform("robot", "ir_right_rear",  now, tf_right_rear);

        pub_ir = nh.advertise<sensor_msgs::PointCloud2>("", 10);
        sub_distance = nh.subscribe ("/sensors/ir/distances", 10, &IrPointsNode::distanceCallback, this);
    }

    static pcl::PointXYZ toPCL(const tf::Vector3& v)
    {
        return pcl::PointXYZ(v.x(), v.y(), v.z());
    }

    void distanceCallback(const sensors::Distance::ConstPtr &distance)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = 1;
        cloud.height = 6;

        cloud.push_back(toPCL(tf_front       * tf::Vector3(distance->front       , 0, 0)));
        cloud.push_back(toPCL(tf_rear        * tf::Vector3(distance->rear        , 0, 0)));
        cloud.push_back(toPCL(tf_left_front  * tf::Vector3(distance->left_front  , 0, 0)));
        cloud.push_back(toPCL(tf_left_rear   * tf::Vector3(distance->left_rear   , 0, 0)));
        cloud.push_back(toPCL(tf_right_front * tf::Vector3(distance->right_front , 0, 0)));
        cloud.push_back(toPCL(tf_right_rear  * tf::Vector3(distance->right_rear  , 0, 0)));

        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud, ros_cloud);
        ros_cloud.header.frame_id = "robot";
        ros_cloud.header.stamp = distance->header.stamp;
        pub_ir.publish(ros_cloud);
    }

    void run()
    {
        ros::spin();
    }
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "ir_points_node");
    IrPointsNode ir_points_node;
    ir_points_node.run();
}

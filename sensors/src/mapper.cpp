#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensors/Distance.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

class IrScanner
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_distance;
    ros::Publisher pub_ir;

    tf::StampedTransform tf_front;
    tf::StampedTransform tf_back;
    tf::StampedTransform tf_left_front;
    tf::StampedTransform tf_left_rear;
    tf::StampedTransform tf_right_front;
    tf::StampedTransform tf_right_rear;

public:
    IrScanner()
        : nh("~")
    {
        ros::Time now(0); //get latest

        tf::TransformListener tfl;
        tfl.lookupTransform("sensors/ir/front",       "robot", now, tf_front);
        tfl.lookupTransform("sensors/ir/back",        "robot", now, tf_back);
        tfl.lookupTransform("sensors/ir/left_front",  "robot", now, tf_left_front);
        tfl.lookupTransform("sensors/ir/left_rear",   "robot", now, tf_left_rear);
        tfl.lookupTransform("sensors/ir/right_front", "robot", now, tf_right_front);
        tfl.lookupTransform("sensors/ir/right_rear",  "robot", now, tf_right_rear);

        pub_ir = nh.advertise<sensor_msgs::PointCloud2>("", 10);
        sub_distance = nh.subscribe ("/sensors/distance", 10, &IrScanner::distanceCallback, this);
    }

    static pcl::PointXYZ toPCL(const tf::Vector3& v)
    {
        return pcl::PointXYZ(v.x(), v.y(), v.z());
    }

    void distanceCallback(const sensors::Distance::ConstPtr &distance)
    {
        //TODO: really we would like to have gotten the time from somewhere else
        ros::Time now = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = 1;
        cloud.height = 6;

        cloud.push_back(toPCL(tf_front        * tf::Vector3(0, distance->front        , 0)));
        cloud.push_back(toPCL(tf_back         * tf::Vector3(0, distance->rear         , 0)));
        cloud.push_back(toPCL(tf_left_front   * tf::Vector3(0, distance->left_front   , 0)));
        cloud.push_back(toPCL(tf_left_rear    * tf::Vector3(0, distance->left_rear    , 0)));
        cloud.push_back(toPCL(tf_right_front  * tf::Vector3(0, distance->right_front  , 0)));
        cloud.push_back(toPCL(tf_right_rear   * tf::Vector3(0, distance->right_rear   , 0)));

        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud, ros_cloud);
        ros_cloud.header.frame_id = "robot";
        ros_cloud.header.stamp = now;
        pub_ir.publish(ros_cloud);
    }

    void run()
    {
        ros::spin();
    }
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "ir_scanner_node");
    IrScanner ir_scanner_node;
    ir_scanner_node.run();
}

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensors/Distance.h>
#include <sensors/SensorClouds.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string>

class IrPointsNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_distance;
    ros::Publisher pub_ir;
    ros::Publisher pub_ir_point_clouds;


    tf::StampedTransform tf_front;
    tf::StampedTransform tf_rear;
    tf::StampedTransform tf_left_front;
    tf::StampedTransform tf_left_rear;
    tf::StampedTransform tf_right_front;
    tf::StampedTransform tf_right_rear;
    double cutoff_front;
    double cutoff_rear;
    double cutoff_left_front;
    double cutoff_left_rear;
    double cutoff_right_front;
    double cutoff_right_rear;

    std::string* transform_ids;

    int counter;

    tf::TransformListener tfl;

public:
    IrPointsNode()
        : nh("~")
    {
        ros::Time now(0); //get latest

        while (!tfl.waitForTransform("ir_front", "robot", now, ros::Duration(1)))
            ROS_ERROR("Couldn't find transform from 'robot' to 'ir_front', retrying...");

        tfl.lookupTransform("robot", "ir_front",       now, tf_front);
        tfl.lookupTransform("robot", "ir_rear",        now, tf_rear);
        tfl.lookupTransform("robot", "ir_left_front",  now, tf_left_front);
        tfl.lookupTransform("robot", "ir_left_rear",   now, tf_left_rear);
        tfl.lookupTransform("robot", "ir_right_front", now, tf_right_front);
        tfl.lookupTransform("robot", "ir_right_rear",  now, tf_right_rear);

        ros::NodeHandle nhp("/sensors/ir/max_distance_cutoff");
        nhp.getParam("front",       cutoff_front);
        nhp.getParam("rear",        cutoff_rear);
        nhp.getParam("left_front",  cutoff_left_front);
        nhp.getParam("left_rear",   cutoff_left_rear);
        nhp.getParam("right_front", cutoff_right_front);
        nhp.getParam("right_rear",  cutoff_right_rear);

        transform_ids = new std::string[6];
        transform_ids[0] = "ir_front";
        transform_ids[1] = "ir_rear";
        transform_ids[2] = "ir_left_front";
        transform_ids[3] = "ir_left_rear";
        transform_ids[4] = "ir_right_front";
        transform_ids[5] = "ir_right_rear";

        pub_ir = nh.advertise<sensor_msgs::PointCloud2>("", 10);
        pub_ir_point_clouds = nh.advertise<sensors::SensorClouds>("/sensors/ir/point_clouds", 10);
        sub_distance = nh.subscribe ("/sensors/ir/distances", 10, &IrPointsNode::distanceCallback, this);
    }

    static pcl::PointXYZ toPCL(const tf::Vector3& v)
    {
        return pcl::PointXYZ(v.x(), v.y(), v.z());
    }

    void distanceCallback(const sensors::Distance::ConstPtr &distance)
    {
        std::vector<u_int8_t> hasEndpoints;
        sensors::SensorClouds sensor_clouds;
        std::vector<sensor_msgs::PointCloud2> pointClouds;
        std::vector<pcl::PointXYZ> ps;

        if (distance->front <= cutoff_front){
            ps.push_back(toPCL(tf::Vector3(distance->front      ,0,0)));
            hasEndpoints.push_back(1);
        }else  {
            ps.push_back(toPCL(tf::Vector3(cutoff_front      ,0,0)));
            hasEndpoints.push_back(0);
        }

        if (distance->rear       <= cutoff_rear       ){
            ps.push_back(toPCL(tf::Vector3(distance->rear       ,0,0)));
            hasEndpoints.push_back(1);
        }else  {
            ps.push_back(toPCL(tf::Vector3(cutoff_rear      ,0,0)));
            hasEndpoints.push_back(0);
        }

        if (distance->left_front <= cutoff_left_front ){
            ps.push_back(toPCL(tf::Vector3(distance->left_front ,0,0)));
            hasEndpoints.push_back(1);
        }else  {
            ps.push_back(toPCL(tf::Vector3(cutoff_left_front      ,0,0)));
            hasEndpoints.push_back(0);
        }

        if (distance->left_rear  <= cutoff_left_rear  ) {
            ps.push_back(toPCL(tf::Vector3(distance->left_rear  ,0,0)));
            hasEndpoints.push_back(1);
        } else  {
            ps.push_back(toPCL(tf::Vector3(cutoff_left_rear      ,0,0)));
            hasEndpoints.push_back(0);
        }

        if (distance->right_front<= cutoff_right_front) {
            ps.push_back(toPCL(tf::Vector3(distance->right_front,0,0)));
            hasEndpoints.push_back(1);
        } else  {
            ps.push_back(toPCL(tf::Vector3(cutoff_right_front      ,0,0)));
            hasEndpoints.push_back(0);
        }

        if (distance->right_rear <= cutoff_right_rear ){
            ps.push_back(toPCL(tf::Vector3(distance->right_rear ,0,0)));
            hasEndpoints.push_back(1);
        }else  {
            ps.push_back(toPCL(tf::Vector3(cutoff_right_rear      ,0,0)));
            hasEndpoints.push_back(0);
        }

        for (int i=0; i<ps.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> temp_cloud;
            temp_cloud.height = temp_cloud.width = 1;
            temp_cloud.push_back(ps[i]);

            sensor_msgs::PointCloud2 temp_ros_cloud;
            pcl::toROSMsg(temp_cloud, temp_ros_cloud);
            temp_ros_cloud.header.frame_id = transform_ids[i];
            temp_ros_cloud.header.stamp = distance->header.stamp;
            pointClouds.push_back(temp_ros_cloud);

        }

        sensor_clouds.hasEndpoint = hasEndpoints;
        sensor_clouds.point_clouds = pointClouds;
        pub_ir_point_clouds.publish(sensor_clouds);

        ps.clear();
        if (distance->front <= cutoff_front) ps.push_back(toPCL(tf_front      *tf::Vector3(distance->front      ,0,0)));
        if (distance->rear       <= cutoff_rear       ) ps.push_back(toPCL(tf_rear       *tf::Vector3(distance->rear       ,0,0)));
        if (distance->left_front <= cutoff_left_front ) ps.push_back(toPCL(tf_left_front *tf::Vector3(distance->left_front ,0,0)));
        if (distance->left_rear  <= cutoff_left_rear  ) ps.push_back(toPCL(tf_left_rear  *tf::Vector3(distance->left_rear  ,0,0)));
        if (distance->right_front<= cutoff_right_front) ps.push_back(toPCL(tf_right_front*tf::Vector3(distance->right_front,0,0)));
        if (distance->right_rear <= cutoff_right_rear ) ps.push_back(toPCL(tf_right_rear *tf::Vector3(distance->right_rear ,0,0)));

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = 1;
        cloud.height = ps.size();
        for (int i=0; i<ps.size(); i++) {
            cloud.push_back(ps[i]);
        }

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

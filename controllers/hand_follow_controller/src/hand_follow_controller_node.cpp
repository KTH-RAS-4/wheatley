
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <pcl/filters/crop_box.h>
#include <ctime>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>


ros::Publisher pub;
ros::Publisher pubTwist;
ros::Publisher pubPose;


geometry_msgs::PoseStamped constructPoseStampedMsg(float xPos, float yPos, float angle)
{
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.frame_id = "/camera_depth_frame";
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.pose.position.x = 0;
        poseMsg.pose.position.y = 0;
        poseMsg.pose.position.z = 0;
        tf::Quaternion quat = tf::createQuaternionFromYaw(angle);
        tf::quaternionTFToMsg(quat, poseMsg.pose.orientation);
        return poseMsg;
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::clock_t start;
    start = std::clock();
    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud_input);

    pcl::CropBox<pcl::PointXYZRGB> cropFilter;
    cropFilter.setInputCloud (cloud_input);

    Eigen::Vector4f minPoint;
    minPoint[0]=-2;
    minPoint[1]=-.15;
    minPoint[2]=0;
    Eigen::Vector4f maxPoint;
    maxPoint[0]=2;
    maxPoint[1]=.10;
    maxPoint[2]=1;
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);

    cropFilter.filter (*cloud_filtered);

    //Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);

    double distance = sqrt(pow(centroid[2],2) + pow(centroid[0],2));
    double angle = -tan(centroid[0]/centroid[2]);
    double travel_speed = 0;
    //if (distance > 0.5)
    //{
        travel_speed = 0.3*(1+(distance-0.5));
    //}
    ROS_INFO("%f angle: %f", travel_speed, angle);
    geometry_msgs::Twist twi;
    twi.linear.x = travel_speed;
    twi.angular.z = angle;
    pubTwist.publish(twi);
    pubPose.publish(constructPoseStampedMsg(centroid[0], centroid[2], angle));

    ROS_INFO("%f", centroid[2]);

    if(cloud_filtered->size() == 0)
    {
        return;
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_filtered, *output);

    // Publish the data
    pub.publish (output);
    ROS_INFO("Time %f ms", (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000));
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "hand_follow_controller");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pubTwist = nh.advertise<geometry_msgs::Twist> ("/motor_controller/twist", 1);
  pubPose = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal",30);

  ros::Rate loop_rate(10.0);

  while(ros::ok()){
    //ros::spin();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



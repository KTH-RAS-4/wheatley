
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <pcl/filters/crop_box.h>



ros::Publisher pub;
ros::Publisher pubTwist;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_voxel(new pcl::PCLPointCloud2());
    //pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert to PCL data type
    //pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
   /* pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.03, 0.03, 0.03);
    sor.filter (*cloud_voxel);*/

    pcl::fromROSMsg(*cloud_msg, *cloud_input);


    //pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
    pcl::CropBox<pcl::PointXYZRGB> cropFilter;
    cropFilter.setInputCloud (cloud_input);

    Eigen::Vector4f minPoint;
    minPoint[0]=-2;  // define minimum point x
    minPoint[1]=-.15;  // define minimum point y
    minPoint[2]=0;  // define minimum point z
    Eigen::Vector4f maxPoint;
    maxPoint[0]=2;  // define max point x
    maxPoint[1]=.10;  // define max point y
    maxPoint[2]=1;  // define max point z
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);

    cropFilter.filter (*cloud_filtered);

    //Compute centroid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_centroid(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::fromPCLPointCloud2(cloud_filtered, *cloud_centroid);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);

    double distance = sqrt(pow(centroid[2],2) + pow(centroid[0],2));
    double angle = tan(centroid[0]/centroid[2]);
    double travel_speed = 0;
    if (distance > 0.5)
    {
        travel_speed = 0.3*(distance-0.5);
    }
    ROS_INFO("%f angle: %f", travel_distance, angle);
    geometry_msgs::Twist twi;
    twi.linear.x = travel_speed;
    twi.angular.z = 0;
    pubTwist.publish(twi);
    // Convert to ROS data type
    //sensor_msgs::PointCloud2 output;
    /*pcl_conversions::fromPCL(*cloud_filtered, output);*/
    //pcl::toROSMsg(*cloud_filtered, output);

    // Publish the data
    //pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pubTwist = nh.advertise<geometry_msgs::Twist> ("/motor_controller/twist", 1);

  ros::Rate loop_rate(10.0);

  while(ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



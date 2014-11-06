#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("Start of Callback");
    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud_input);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_input);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

/*
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.2 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the inliers
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);
      std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

      // Create the filtering object
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);
      i++;
    }*/

    // kd-tree object for searches.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud);

    // Color-based region growing clustering object.
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
    clustering.setInputCloud(cloud_filtered);
    clustering.setSearchMethod(kdtree);
    // Here, the minimum cluster size affects also the postprocessing step:
    // clusters smaller than this will be merged with their neighbors.
    clustering.setMinClusterSize(100);
    // Set the distance threshold, to know which points will be considered neighbors.
    clustering.setDistanceThreshold(10);
    // Color threshold for comparing the RGB color of two points.
    clustering.setPointColorThreshold(6);
    // Region color threshold for the postprocessing step: clusters with colors
    // within the threshold will be merged in one.
    clustering.setRegionColorThreshold(5);

    std::vector <pcl::PointIndices> clusters;
    clustering.extract(clusters);

    // For every cluster...
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {

        // ...add all its points to a new cloud...
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(cloud_filtered, i->indices, cluster);
    }



    // Convert to ROS data type
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_filtered, *output);

    pub.publish(output);
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_detection_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/object_detection/cloud", 1);

  ros::Rate loop_rate(10.0);

  while(ros::ok()){
    //ros::spin();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



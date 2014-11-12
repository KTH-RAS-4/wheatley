#include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

ros::Publisher pub_plane;
ros::Publisher pub_objects;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("Start of Callback");
    // Container for original & filtered data
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    //pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());//, cloud_p (new pcl::PointCloud<pcl::PointT>), cloud_f (new pcl::PointCloud<pcl::PointT>);

    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(cloud->size() == 0)
        return;

    /** Segmentation */
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::VoxelGrid<PointT> sor;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_downsampled (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
    reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    // Create the filtering object
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_downsampled);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud_downsampled);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered2, output);
    pub_objects.publish(output);

    sensor_msgs::PointCloud2 output_plane;
    pcl::toROSMsg(*cloud_filtered, output_plane);
    pub_plane.publish(output);
}



int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "object_detection_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);


    // Create a ROS publisher for the output point cloud
    pub_plane = nh.advertise<sensor_msgs::PointCloud2> ("/object_detection/plane", 1);
    pub_objects = nh.advertise<sensor_msgs::PointCloud2> ("/object_detection/objects", 1);


    ros::spin();
    return 0;
}



#include <ros/ros.h>

#include <vision_msgs/PreprocessedClouds.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub_plane;
ros::Publisher pub_others;
ros::Publisher pub_preprocessed;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(cloud->size() == 0)
        return;

    /** Segmentation */
    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::VoxelGrid<PointT> vgrid;
    pcl::StatisticalOutlierRemoval<PointT> sor;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_downsampled (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_cleaned (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

    // Create the filtering object
    vgrid.setInputCloud (cloud);
    vgrid.setLeafSize (0.005f, 0.005f, 0.005f);
    vgrid.filter (*cloud_downsampled);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud_downsampled);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);

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


    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_cleaned);

    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    sor.setInputCloud (cloud_cleaned);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    //sor.filter (*cloud_projected);
    sor.filter (*cloud_filtered2);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_plane;
    pcl::toROSMsg(*cloud_plane, output_plane);
    pub_plane.publish(output_plane);

    sensor_msgs::PointCloud2 output_others;
    pcl::toROSMsg(*cloud_filtered2, output_others);
    pub_others.publish(output_others);

    vision_msgs::PreprocessedClouds output_preprocessed;
    output_preprocessed.plane = output_plane;
    output_preprocessed.others = output_others;
    pub_preprocessed.publish(output_preprocessed);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cloud_preprocessing_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);


    // Create a ROS publisher for the output point cloud
    pub_plane = nh.advertise<sensor_msgs::PointCloud2> ("/object_detection/plane", 1);
    pub_others = nh.advertise<sensor_msgs::PointCloud2> ("/object_detection/others", 1);
    pub_preprocessed = nh.advertise<vision_msgs::PreprocessedClouds> ("/object_detection/preprocessed", 1);

    ros::spin();
    return 0;
}

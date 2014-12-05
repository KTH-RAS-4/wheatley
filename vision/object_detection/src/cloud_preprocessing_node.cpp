#include <ros/ros.h>

#include <vision_msgs/PreprocessedClouds.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZRGB PointT;

class CloudPreprocessingNode {


private:
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Publisher pub_plane;
    ros::Publisher pub_others;
    ros::Publisher pub_preprocessed;

    tf::TransformListener tf_;

public:
    CloudPreprocessingNode() {

        while (!tf_.waitForTransform("camera_rgb_optical_frame", "map", ros::Time::now(), ros::Duration(.1)))
            ROS_ERROR_STREAM("Couldn't find transform from 'camera_rgb_optical_frame' to 'map', retrying...");

        // Create a ROS subscriber for the input point cloud
        sub = nh.subscribe ("/camera/depth_registered/points", 1, &CloudPreprocessingNode::cloud_cb, this);


        // Create a ROS publisher for the output point cloud
        pub_plane = nh.advertise<sensor_msgs::PointCloud2> ("/object_detection/plane", 1);
        pub_others = nh.advertise<sensor_msgs::PointCloud2> ("/object_detection/others", 1);
        pub_preprocessed = nh.advertise<vision_msgs::PreprocessedClouds> ("/object_detection/preprocessed", 1);

        ROS_INFO("Constructed");
    }

    ~CloudPreprocessingNode() {

    }

    void run() {
        ROS_INFO("Running");
        ros::spin();
    }

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

        if(!tf_.waitForTransform("camera_rgb_optical_frame", "map", cloud_msg->header.stamp, ros::Duration(0.1)))
            return;

        pcl::fromROSMsg(*cloud_msg, *cloud);

        if(cloud->size() == 0)
            return;

        /** Segmentation */
        // All the objects needed
        pcl::PassThrough<PointT> pass;
        pcl::VoxelGrid<PointT> vgrid;
        pcl::StatisticalOutlierRemoval<PointT> sor;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

        // Datasets
        pcl::PointCloud<PointT>::Ptr cloud_downsampled (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_pass_y (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_transformed (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_plane_filtered (new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

        // Create the filtering object
        vgrid.setInputCloud (cloud);
        vgrid.setLeafSize (0.01f, 0.01f, 0.01f);
        vgrid.filter (*cloud_downsampled);

        if(cloud_downsampled->size() == 0)
            return;

        pcl::PassThrough<PointT> passY;
        passY.setInputCloud (cloud_downsampled);
        passY.setFilterFieldName ("y");
        passY.setFilterLimits (0, 2);
        passY.filter (*cloud_pass_y);

        if(cloud_pass_y->size() == 0)
            return;

        sensor_msgs::PointCloud2 cloud_msg_pass_y;
        pcl::toROSMsg(*cloud_pass_y, cloud_msg_pass_y);

        sensor_msgs::PointCloud cloud_legacy;
        sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg_pass_y, cloud_legacy);

        sensor_msgs::PointCloud cloud_msg_transformed;
        tf_.transformPointCloud("map", cloud_msg->header.stamp, cloud_legacy, "camera_rgb_optical_frame", cloud_msg_transformed);

        sensor_msgs::PointCloud2 cloud2_transformed;
        sensor_msgs::convertPointCloudToPointCloud2(cloud_msg_transformed, cloud2_transformed);

        // Convert to PCL data type
        pcl::fromROSMsg(cloud2_transformed, *cloud_transformed);

        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud_transformed);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.015, 1.5);
        pass.filter (*cloud_filtered);
        pass.setNegative(true);
        pass.filter(*cloud_plane);

        if(cloud_plane->size() == 0)
            return;


        pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, 0.01)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, -0.02)));
        // build the filter
        pcl::ConditionalRemoval<PointT> condrem (range_cond);
        condrem.setInputCloud (cloud_plane);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*cloud_plane_filtered);

        if(cloud_plane_filtered->size() == 0)
            return;

        sensor_msgs::PointCloud2 output_plane;
        pcl::toROSMsg(*cloud_plane_filtered, output_plane);
        output_plane.header.stamp = cloud_msg->header.stamp;
        pub_plane.publish(output_plane);


        if(cloud_filtered->size() == 0)
            return;

        sor.setInputCloud (cloud_filtered);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered2);

        if(cloud_filtered2->size() == 0)
            return;

        sensor_msgs::PointCloud2 output_others;
        pcl::toROSMsg(*cloud_filtered2, output_others);
        output_others.header.stamp = cloud_msg->header.stamp;
        pub_others.publish(output_others);


        vision_msgs::PreprocessedClouds output_preprocessed;
        output_preprocessed.plane = output_plane;
        output_preprocessed.others = output_others;
        pub_preprocessed.publish(output_preprocessed);

        /*

        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;

        if(cloud_filtered->empty())
            return;

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
        pub_preprocessed.publish(output_preprocessed);*/
    }
};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cloud_preprocessing_node");
    CloudPreprocessingNode node;
    node.run();
    return 0;
}

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

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

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ PointT;

ros::Publisher pub_plane;
ros::Publisher pub_objects;
ros::Publisher marker_pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("Start of Callback");
    // Container for original & filtered data
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    //pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());//, cloud_p (new pcl::PointCloud<pcl::PointT>), cloud_f (new pcl::PointCloud<pcl::PointT>);

    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(cloud->size() == 0)
        return;

    /** Set up transform */
    //"camera_link" transform
    ros::Time now = ros::Time::now();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.3));
    tf::Quaternion q;
    q.setRPY(-3*M_PI/180, 20*M_PI/180, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, now, "robot", "camera_link"));

    /** Set up markers */
    visualization_msgs::Marker points;
    points.header.frame_id = "/camera_depth_optical_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.05 ;
    points.scale.y = 0.05;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    /** Segmentation */
    // All the objects needed
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
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

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

    // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud (cloud_filtered2);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.04); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered2);
    ec.extract (cluster_indices);


    int j = 0;
    sensor_msgs::PointCloud2 output_object;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>(*cloud_filtered2, it->indices));
        pcl::toROSMsg(*cloud_cluster, output_object);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_filtered2, it->indices, centroid);

        geometry_msgs::Point p;
        p.x = centroid[0];
        p.y = centroid[1];
        p.z = centroid[2];
        points.points.push_back(p);

        std::cout << "Centroid:" << centroid << std::endl;
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;
    }
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered2, output);
    pub_plane.publish(output);

    pub_objects.publish(output_object);

    marker_pub.publish(points);


    /*sensor_msgs::PointCloud2 output_plane;
    pcl::toROSMsg(*cloud_filtered, output_plane);
    pub_plane.publish(output_plane);*/
/*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    visualization_msgs::Marker points;
    points.header.frame_id = "/camera_depth_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "point";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.01;
    points.scale.y = 0.01;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    geometry_msgs::Point c;
    c.x = 0;
    c.y = 0;
    c.z = 0;
    points.points.push_back(c);

    int j = 0;
    sensor_msgs::PointCloud2 output_object;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>(*cloud_filtered, it->indices));
        pcl::toROSMsg(*cloud_cluster, output_object);

        /*for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;*

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_filtered, it->indices, centroid);

        geometry_msgs::Point p;
        p.x = centroid[0];
        p.y = centroid[1];
        p.z = centroid[2];
        points.points.push_back(p);

        std::cout << "Centroid:" << centroid << std::endl;
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;
    }

    marker_pub.publish(points);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    pub_objects.publish(output);

    pub_plane.publish(output_object);


    /*sensor_msgs::PointCloud2 output_plane;
    pcl::toROSMsg(*cloud_filtered, output_plane);
    pub_plane.publish(output_plane);*/
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
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    ros::spin();
    return 0;
}



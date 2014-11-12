#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <ras_arduino_msgs/Object.h>
#include <ras_arduino_msgs/Objects.h>

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

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub_plane;
ros::Publisher pub_objects;
ros::Publisher marker_pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //ROS_INFO("Start of Callback");
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
    //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

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
    //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

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
    ras_arduino_msgs::Objects objects;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>(*cloud_filtered2, it->indices));
        //pcl::toROSMsg(*cloud_cluster, output_object);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_filtered2, it->indices, centroid);

        geometry_msgs::Point p;
        p.x = centroid[0];
        p.y = centroid[1];
        p.z = centroid[2];
        points.points.push_back(p);

        Eigen::Vector3f color(0,0,0);
        for(int k = 0; k < cloud_cluster->size(); k++) {
            Eigen::Vector3i col = cloud_cluster->points[k].getRGBVector3i();
            color[0] += col[0];
            color[1] += col[1];
            color[2] += col[2];
        }
        color[0] /= (float) cloud_cluster->size();
        color[1] /= (float) cloud_cluster->size();
        color[2] /= (float) cloud_cluster->size();

        ras_arduino_msgs::Object ob;
        ob.r = color[0];
        ob.g = color[1];
        ob.b = color[2];
        objects.objects.push_back(ob);

        std::cout << "Centroid:" << centroid << std::endl;
        std::cout << "Color:" << color << std::endl;
        j++;
    }
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered2, output);
    pub_plane.publish(output);

    pub_objects.publish(objects);

    marker_pub.publish(points);
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
    pub_objects = nh.advertise<ras_arduino_msgs::Objects> ("/object_detection/objects", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    ros::spin();
    return 0;
}



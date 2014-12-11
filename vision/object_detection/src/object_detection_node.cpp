#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>

#include <vision_msgs/Object.h>
#include <vision_msgs/Objects.h>
#include <vision_msgs/PreprocessedClouds.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZRGB PointT;


class ObjectDetectionNode {

private:
    ros::NodeHandle nh;
    ros::Publisher pub_objects;
    ros::Publisher marker_pub;
    ros::Subscriber sub;

    visualization_msgs::Marker points;
    visualization_msgs::Marker walls;

    tf::TransformListener tfl;

public:
    ObjectDetectionNode() {
        ros::Time now(0);

        while (!tfl.waitForTransform("camera_depth_optical_frame", "world", now, ros::Duration(1)))
            ROS_ERROR("Couldn't find transform from 'robot' to 'ir_front', retrying...");

        sub = nh.subscribe ("/object_detection/preprocessed", 1, &ObjectDetectionNode::preprocessed_cb, this);

        pub_objects = nh.advertise<vision_msgs::Objects> ("/object_detection/objects", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/object_detection/markers", 10);


        /** Set up markers */
        points.header.frame_id = walls.header.frame_id = "/map";
        points.ns = walls.ns = "points_and_lines";
        points.action = walls.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = walls.pose.orientation.w = 1.0;

        points.id = 0;
        walls.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        walls.type = visualization_msgs::Marker::LINE_LIST;

        points.scale.x = 0.05 ;
        points.scale.y = 0.05;

        walls.scale.x = 0.01;

        points.color.g = 1.0;
        points.color.a = 1.0;

        walls.color.r = 1.0;
        walls.color.a = 1.0;
    }

    ~ObjectDetectionNode() {

    }

    void preprocessed_cb (const vision_msgs::PreprocessedCloudsConstPtr& preprocessed_msg)
    {

        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_others (new pcl::PointCloud<PointT>);

        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

        points.header.stamp = walls.header.stamp= preprocessed_msg->plane.header.stamp;
        points.points.clear();
        walls.points.clear();

        pcl::fromROSMsg(preprocessed_msg->plane, *cloud_plane);
        pcl::fromROSMsg(preprocessed_msg->others, *cloud_others);

        if(cloud_plane->size() == 0 || cloud_others->size() == 0)
            return;

        //ROS_INFO("Clouds are not empty");
        //Generate clusters
        std::vector<pcl::PointIndices> cluster_indices;
        /*pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.02);
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (150);
        //tree->setInputCloud (cloud_others);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_others);
        ec.extract (cluster_indices);*/

        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud (cloud_others);
        reg.setSearchMethod (tree);
        reg.setDistanceThreshold (0.07);
        reg.setPointColorThreshold (15);
        reg.setRegionColorThreshold (15);
        reg.setMinClusterSize (40);

        reg.extract (cluster_indices);

        ROS_INFO("Extract done");
        if(cloud_plane->empty() || cluster_indices.empty())
            return;
        try {
            tree->setInputCloud(cloud_plane);
        } catch ( ... ) {
            return;
        }

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        int j = 0;
        vision_msgs::Objects objects;
        objects.header = preprocessed_msg->plane.header;
        objects.header.frame_id = "/map";

        /*if (!tfl.waitForTransform("camera_rgb_optical_frame", "map", objects.header.stamp, ros::Duration(1))) {
            ROS_ERROR("Couldn't find transform from 'camera_rgb_optical_frame' to 'map', retrying...");
            return;
        }*/

        //ROS_INFO("Starting clustering");
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            //ROS_INFO("Loop");
            pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>(*cloud_others, it->indices));

            if(cloud_cluster->size() == 0)
                continue;
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_others, it->indices, centroid);

            PointT centroid_point;
            centroid_point.x = centroid[0];
            centroid_point.y = centroid[1];
            centroid_point.z = centroid[2];

            if(tree->nearestKSearch(centroid_point, 1,  pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {

                if(pointNKNSquaredDistance[0] > 0.05) {

                    //std::cout << "Distance:" << pointNKNSquaredDistance[0] << std::endl;

                    /*geometry_msgs::Point w;
                    w.x = cloud_plane->points[ pointIdxNKNSearch[0] ].x;
                    w.y = cloud_plane->points[ pointIdxNKNSearch[0] ].y;
                    w.z = cloud_plane->points[ pointIdxNKNSearch[0] ].z;

                    geometry_msgs::Point w2;
                    w2.x = centroid[0];
                    w2.y = centroid[1];
                    w2.z = centroid[2];

                    walls.points.push_back(w);
                    walls.points.push_back(w2);*/
                    continue;
                }
            }

            //Get mean color
            Eigen::Vector3f color(0,0,0);
            for(int k = 0; k < cloud_cluster->size(); ) {
                Eigen::Vector3i col = cloud_cluster->points[k].getRGBVector3i();
                color[0] += col[0];
                color[1] += col[1];
                color[2] += col[2];
                k++;
            }
            color[0] /= (float) cloud_cluster->size();
            color[1] /= (float) cloud_cluster->size();
            color[2] /= (float) cloud_cluster->size();

            geometry_msgs::PointStamped p;
            geometry_msgs::PointStamped p_transformed;
            p.header = preprocessed_msg->plane.header;


            //Generate output and marker point
            vision_msgs::Object ob;

            p.point.x = centroid[0];
            p.point.y = centroid[1];
            p.point.z = centroid[2];

            //tfl.transformPoint("map", p, p_transformed);

            ob.x = p.point.x;
            ob.y = p.point.y;
            ob.z = p.point.z;
            ob.r = color[0];
            ob.g = color[1];
            ob.b = color[2];

            points.points.push_back(p.point);
            objects.objects.push_back(ob);

            //std::cout << "Centroid:" << centroid << std::endl;
            //std::cout << "Color:" << color << std::endl;
            j++;
        }
        //ROS_INFO("Clustering done");

        pub_objects.publish(objects);
        std::cout << "Published " << objects.objects.size() << " objects" << std::endl;

        visualization_msgs::MarkerArray marker;
        marker.markers.push_back(points);
        marker.markers.push_back(walls);
        marker_pub.publish(marker);
    }
    void run() {
        ros::spin();
    }
};



int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "object_detection_node");
    ObjectDetectionNode node;
    ros::Duration(5).sleep();
    node.run();
    return 0;
}



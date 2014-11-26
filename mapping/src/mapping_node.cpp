#include <iostream>
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <list>

#include <vision_msgs/PreprocessedClouds.h>
#include <vision_msgs/Objects.h>
#include <vision_msgs/Object.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ras_arduino_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <map>

#include <sensors/SensorClouds.h>

#include <tf/transform_listener.h>

#include <occupancy_grid_utils/ray_tracer.h>
#include <ros/wall_timer.h>

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;

using std::vector;
using std::string;
using std::list;
using std::abs;

typedef boost::mutex::scoped_lock Lock;
typedef boost::shared_ptr<gu::LocalizedCloud> CloudPtr;
typedef boost::shared_ptr<gu::LocalizedCloud const> CloudConstPtr;
typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;


class MapNode {
private:

    ros::NodeHandle handle;
    ros::Subscriber sub_pointcloud;
    ros::Subscriber sub_objects;
    ros::Subscriber sub_ir;



    /****************************************
     * Params
     ****************************************/

    ros::NodeHandle nh_;
    unsigned history_length_;
    double resolution_;
    string fixed_frame_;
    string sensor_frame_;
    double grid_construction_interval_;
    double local_grid_size_;

    /****************************************
     * Associated objects
     ****************************************/

    tf::TransformListener tf_;
    ros::Publisher grid_pub_;
    ros::Publisher object_pub_;
    ros::WallTimer build_grid_timer_;
    boost::mutex mutex_;

    /****************************************
     * State
     ****************************************/

    CloudBuffer clouds_;
    CloudConstPtr last_cloud_;


    /****************************************
     * Stores
     ****************************************/

    gu::OverlayClouds map;
    list<vision_msgs::Object> object_collector;


public:
    MapNode()
    {
        ROS_INFO("Running");
        grid_construction_interval_ = 0.1;
        history_length_ = 10000;
        fixed_frame_ = "map";
        resolution_ = 0.03;
        sensor_frame_ = "robot";
        local_grid_size_ = 5.0;

        grid_pub_ = handle.advertise<nm::OccupancyGrid>("/map", 100);
        object_pub_ = handle.advertise<visualization_msgs::MarkerArray>("/map_objects/", 100);

        //sub_pointcloud = handle.subscribe("/object_detection/preprocessed", 1, &MapNode::mapPointCloud, this);
        sub_ir = handle.subscribe("/sensors/ir/point_clouds", 100, &MapNode::mapIr, this);
        sub_objects = handle.subscribe("/object_recognition/objects", 100, &MapNode::insertObject, this);

        build_grid_timer_ = handle.createWallTimer(ros::WallDuration(grid_construction_interval_), &MapNode::echoGrid, this);

        initGrid();
    }

    ~MapNode()
    {
    }

    void initGrid() {

        ros::Time now(0);

        while (!tf_.waitForTransform("robot", "map", now, ros::Duration(1)))
            ROS_ERROR("Couldn't find transform from 'robot' to 'map', retrying...");

        // Figure out current position
        gm::PoseStamped identity, odom_pose;
        identity.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        identity.header.frame_id = sensor_frame_;
        identity.header.stamp = ros::Time();
        tf_.transformPose(fixed_frame_, identity, odom_pose);

        // Set up map dimensions
        nm::MapMetaData info;
        info.origin.position.x = odom_pose.pose.position.x-local_grid_size_/2;
        info.origin.position.y = odom_pose.pose.position.y-local_grid_size_/2;
        info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
        info.resolution = resolution_;
        info.width = local_grid_size_/resolution_;
        info.height = local_grid_size_/resolution_;

        nm::OccupancyGrid fake_grid;
        fake_grid.info = info;
        map = gu::createCloudOverlay(fake_grid, fixed_frame_, 0.1, 10, 2);
    }

    void echoGrid(const ros::WallTimerEvent& scan) {
        nm::OccupancyGrid::ConstPtr grid = gu::getGrid(map);
        grid_pub_.publish(grid);

        echoObjects();
    }

    void echoObjects() {
        visualization_msgs::MarkerArray object_marker;

        int counter = 0;
        for (list<vision_msgs::Object>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {
            visualization_msgs::Marker marker;
            visualization_msgs::Marker text;
            marker.header.frame_id = text.header.frame_id = "/map";
            marker.ns = text.ns = "objects";
            marker.action = text.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = text.pose.orientation.w = 1.0;

            marker.id = counter++;
            text.id = counter++;

            marker.type = visualization_msgs::Marker::POINTS;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            marker.scale.x = 0.05 ;
            marker.scale.y = 0.05;

            text.scale.z = 0.05;

            marker.color.r = it->r / 255;
            marker.color.g = it->g / 255;
            marker.color.b = it->b / 255;
            marker.color.a = 1.0;

            geometry_msgs::Point p;
            p.x = it->x;
            p.y = it->y;
            p.z = it->z;
            marker.points.push_back(p);
            text.points.push_back(p);
            text.text = it->type;
            object_marker.markers.push_back(marker);
            object_marker.markers.push_back(text);
        }
        object_pub_.publish(object_marker);
    }

    void mapPose(const ras_arduino_msgs::Odometry &msg)
    {

    }

    void insertObject(const vision_msgs::Object &new_object) {
        ROS_INFO("Inserting object");
        for (list<vision_msgs::Object>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {

            //If already inserted
            if(abs(new_object.x - it->x) < 0.03 && abs(new_object.y - it->y) < 0.03 && abs(new_object.z - it->z) < 0.03) {
                ROS_INFO("Already there, differences: %f, %f, %f", abs(new_object.x - it->x), abs(new_object.y - it->y), abs(new_object.z - it->z));
                return;
            }
        }
        ROS_INFO("Inserted new object");
        object_collector.push_back(new_object);

    }

    void mapIr(const sensors::SensorClouds &msg)
    {
        std::vector<sensor_msgs::PointCloud2> clouds = msg.point_clouds;
        std::vector<u_int8_t> hasEndpoint = msg.hasEndpoint;

        ros::Time stamp;

        for(int pc = 0; pc < 6; pc++) {
            sensor_frame_ = clouds[pc].header.frame_id;
            stamp = clouds[pc].header.stamp;
            try {
                // We'll need the transform between sensor and fixed frames at the time when the scan was taken
                if (!tf_.waitForTransform(fixed_frame_, sensor_frame_, clouds[pc].header.stamp, ros::Duration(1.0)))
                {
                    ROS_WARN_STREAM ("Timed out waiting for transform from " << sensor_frame_ << " to "
                                     << fixed_frame_ << " at " << clouds[pc].header.stamp.toSec());
                    return;
                }

                // Figure out current sensor position
                tf::Pose identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));
                tf::Stamped<tf::Pose> odom_pose;
                tf_.transformPose(fixed_frame_, tf::Stamped<tf::Pose> (identity, ros::Time(), sensor_frame_), odom_pose);

                sm::PointCloud sensor_frame_cloud;
                sensor_msgs::convertPointCloud2ToPointCloud(clouds[pc], sensor_frame_cloud);


                // Construct and save LocalizedCloud
                CloudPtr loc_cloud(new gu::LocalizedCloud());
                loc_cloud->hasEndpoint = hasEndpoint[pc];
                loc_cloud->cloud.points = sensor_frame_cloud.points;
                tf::poseTFToMsg(odom_pose, loc_cloud->sensor_pose);
                loc_cloud->header.frame_id = fixed_frame_;

                Lock lock(mutex_);
                gu::addCloud(&map, loc_cloud, hasEndpoint[pc]);

                //last_cloud_=loc_cloud;
                //clouds_.push_back(last_cloud_);
            }
            catch (tf::TransformException& e) {
              ROS_INFO ("Not saving scan due to tf lookup exception: %s",
                        e.what());
            }
        }
        tf::Point identity(0,0,0);
        geometry_msgs::PointStamped identity_msg;
        identity_msg.header.frame_id = "robot";
        identity_msg.header.stamp = stamp;
        tf::pointTFToMsg(identity, identity_msg.point);

        geometry_msgs::PointStamped transformed;
        tf_.transformPoint("map", identity_msg, transformed);

        gu::addKnownFreePoint(&map, transformed.point, 0.12);
    }

    void mapPointCloud(const vision_msgs::PreprocessedClouds &msg)
    {
        sensor_msgs::PointCloud2 cloud = msg.others;
        try {
            // We'll need the transform between sensor and fixed frames at the time when the scan was taken
            if (!tf_.waitForTransform(fixed_frame_, sensor_frame_, msg.header.stamp, ros::Duration(1.0)))
            {
                ROS_WARN_STREAM ("Timed out waiting for transform from " << sensor_frame_ << " to "
                                 << fixed_frame_ << " at " << msg.header.stamp.toSec());
                return;
            }

            // Figure out current sensor position
            tf::Pose identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));
            tf::Stamped<tf::Pose> odom_pose;
            tf_.transformPose(fixed_frame_, tf::Stamped<tf::Pose> (identity, ros::Time(), sensor_frame_), odom_pose);

            sm::PointCloud cloud_legacy;
            sensor_msgs::convertPointCloud2ToPointCloud(cloud, cloud_legacy);

            sm::PointCloud sensor_frame_cloud;
            tf_.transformPointCloud (sensor_frame_, cloud_legacy, sensor_frame_cloud);

            // Construct and save LocalizedCloud
            CloudPtr loc_cloud(new gu::LocalizedCloud());
            loc_cloud->cloud.points = sensor_frame_cloud.points;
            tf::poseTFToMsg(odom_pose, loc_cloud->sensor_pose);
            loc_cloud->header.frame_id = fixed_frame_;
            Lock lock(mutex_);
            last_cloud_=loc_cloud;
        }
        catch (tf::TransformException& e) {
          ROS_INFO ("Not saving scan due to tf lookup exception: %s",
                    e.what());
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_node");
    MapNode mn;
    ros::spin();
    return 0;
}

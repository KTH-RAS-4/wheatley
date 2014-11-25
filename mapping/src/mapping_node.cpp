#include <iostream>
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vision_msgs/PreprocessedClouds.h>
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

typedef boost::mutex::scoped_lock Lock;
typedef boost::shared_ptr<gu::LocalizedCloud> CloudPtr;
typedef boost::shared_ptr<gu::LocalizedCloud const> CloudConstPtr;
typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;


class MapNode
{
private:

    ros::NodeHandle handle;
    ros::Subscriber sub_pointcloud;
    ros::Subscriber sub_pose;
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
    ros::Subscriber scan_sub_;
    ros::Publisher grid_pub_;
    ros::WallTimer build_grid_timer_;
    boost::mutex mutex_;

    /****************************************
     * State
     ****************************************/

    CloudBuffer clouds_;
    CloudConstPtr last_cloud_;


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
        grid_pub_ = handle.advertise<nm::OccupancyGrid>("/map", 1);

        //sub_pointcloud = handle.subscribe("/object_detection/preprocessed", 1, &MapNode::mapPointCloud, this);
        sub_ir = handle.subscribe("/sensors/ir/point_clouds", 1, &MapNode::mapIr, this);
        //sub_pose = handle.subscribe("/sensors/pose", 1, &MapNode::mapPose, this);
        build_grid_timer_ = handle.createWallTimer(ros::WallDuration(grid_construction_interval_), &MapNode::buildGrid, this);
        clouds_ = CloudBuffer(history_length_);
    }

    ~MapNode()
    {
    }

    void buildGrid(const ros::WallTimerEvent& scan) {
        if (last_cloud_) {
            {
            Lock lock(mutex_);
            //clouds_.push_back(last_cloud_);
            last_cloud_.reset();
            }

            ROS_DEBUG_NAMED ("build_grid", "Building grid with %zu scans", clouds_.size());

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
            gu::OverlayClouds overlay = gu::createCloudOverlay(fake_grid, fixed_frame_, 0.1, 10, 2);
            vector<CloudConstPtr> clouds(clouds_.begin(), clouds_.end());
            BOOST_FOREACH  (CloudConstPtr cloud, clouds_) {
                gu::addCloud(&overlay, cloud, cloud->hasEndpoint);
            }


            nm::OccupancyGrid::ConstPtr grid = gu::getGrid(overlay);

            ROS_DEBUG_NAMED ("build_grid", "Done building grid");

            grid_pub_.publish(grid);
        }
    }

    void mapPose(const ras_arduino_msgs::Odometry &msg)
    {

    }

    void mapIr(const sensors::SensorClouds &msg)
    {
        std::vector<sensor_msgs::PointCloud2> clouds = msg.point_clouds;
        std::vector<u_int8_t> hasEndpoint = msg.hasEndpoint;

        for(int pc = 0; pc < 6; pc++) {
            sensor_frame_ = clouds[pc].header.frame_id;
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
                last_cloud_=loc_cloud;
                clouds_.push_back(last_cloud_);
            }
            catch (tf::TransformException& e) {
              ROS_INFO ("Not saving scan due to tf lookup exception: %s",
                        e.what());
            }
        }
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

            // Project scan from sensor frame (which varies over time) to odom (which doesn't)
            /*sm::PointCloud fixed_frame_cloud;
            laser_geometry::LaserProjection projector_;
            projector_.transformLaserScanToPointCloud (fixed_frame_, scan, fixed_frame_cloud, tf_);*/


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

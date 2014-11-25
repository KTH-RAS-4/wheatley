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
        grid_construction_interval_ = 0.3;
        history_length_ = 100;
        fixed_frame_ = "map";
        resolution_ = 0.05;
        sensor_frame_ = "robot";
        local_grid_size_ = 5.0;
        grid_pub_ = handle.advertise<nm::OccupancyGrid>("/map", 1);

        //sub_pointcloud = handle.subscribe("/object_detection/others", 1, &MapNode::mapIr, this);
        sub_ir = handle.subscribe("/sensors/ir/points", 1, &MapNode::mapIr, this);
        //sub_pose = handle.subscribe("/sensors/pose", 1, &MapNode::mapPose, this);
        build_grid_timer_ = handle.createWallTimer(ros::WallDuration(grid_construction_interval_), &MapNode::buildGrid, this);
        clouds_ = CloudBuffer(history_length_);
    }

    ~MapNode()
    {
    }

    void buildGrid(const ros::WallTimerEvent& scan) {
        if (last_cloud_) {
            ROS_INFO ("Build grid");

            {
            Lock lock(mutex_);
            clouds_.push_back(last_cloud_);
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
                //ROS_INFO ("Add cloud");
                gu::addCloud(&overlay, cloud);
            }
            nm::OccupancyGrid::ConstPtr grid = gu::getGrid(overlay);

            ROS_DEBUG_NAMED ("build_grid", "Done building grid");

            grid_pub_.publish(grid);
        }
    }

    void mapPose(const ras_arduino_msgs::Odometry &msg)
    {

    }

    void mapIr(const sensor_msgs::PointCloud2 &msg)
    {
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

            // Now transform back into sensor frame at a single time point
            /*sm::PointCloud sensor_frame_cloud;
            tf_.transformPointCloud (sensor_frame_, scan.header.stamp, fixed_frame_cloud, fixed_frame_,
                                   sensor_frame_cloud);*/


            sm::PointCloud sensor_frame_cloud;
            sensor_msgs::convertPointCloud2ToPointCloud(msg, sensor_frame_cloud);

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

    void mapPointCloud(const vision_msgs::PreprocessedClouds &msg)
    {

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detector_node");
    MapNode mn;
    ros::spin();
    return 0;
}

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
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/file.h>
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

tf::Pose identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));

class MapNode {
private:

    ros::NodeHandle handle;
    ros::Subscriber sub_pointcloud;
    ros::Subscriber sub_objects;
    ros::Subscriber sub_ir;
    ros::Subscriber sub_cloud;
    ros::Subscriber sub_executor_state;




    /****************************************
     * Params
     ****************************************/

    ros::NodeHandle nh_;
    unsigned history_length_;
    double resolution_;
    string fixed_frame_;
    string robot_frame;
    double robot_outer_diameter;
    double mark_pose_explored_rate;
    double grid_construction_interval_;
    double local_grid_size_;

    nm::OccupancyGrid fake_grid;


    /****************************************
     * Associated objects
     ****************************************/

    tf::TransformListener tf_;
    ros::Publisher grid_pub_;
    ros::Publisher temp_grid_pub_;
    ros::Publisher object_pub_;
    ros::WallTimer build_grid_timer_;
    ros::WallTimer mark_pose_explored_timer_;
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
    gu::OverlayClouds temp_map;
    list<nm::OccupancyGrid::ConstPtr> map_collector;
    list<vision_msgs::Object> object_collector;


    int current_iteration;
    bool isMapping;


public:
    MapNode()
    {
        ROS_INFO("Running");
        grid_construction_interval_ = 0.1;
        mark_pose_explored_rate = 10;
        history_length_ = 10000;
        robot_outer_diameter = 0.22;//0.255;
        fixed_frame_ = "map";
        resolution_ = 0.01;
        robot_frame = "robot";
        local_grid_size_ = 15.0;

        grid_pub_ = handle.advertise<nm::OccupancyGrid>("/map", 100);
        temp_grid_pub_ = handle.advertise<nm::OccupancyGrid>("/temp_map", 100);


        //sub_pointcloud = handle.subscribe("/object_detection/preprocessed", 1, &MapNode::mapPointCloud, this);
        sub_ir = handle.subscribe("/sensors/ir/point_clouds", 100, &MapNode::mapIr, this);
        sub_cloud = handle.subscribe("/object_detection/preprocessed", 100, &MapNode::mapCloud, this);
        sub_executor_state = handle.subscribe("/executor/state", 1, &MapNode::executorState, this);


        mark_pose_explored_timer_ = handle.createWallTimer(ros::WallDuration(1/mark_pose_explored_rate), &MapNode::mapMarkPoseExplored, this);
        build_grid_timer_ = handle.createWallTimer(ros::WallDuration(grid_construction_interval_), &MapNode::echoGrid, this);

        init();
        isMapping = true;
    }

    ~MapNode()
    {
    }

    void init()
    {
        ros::Time now(0);

        while (!tf_.waitForTransform(robot_frame, fixed_frame_, now, ros::Duration(1)))
            ROS_ERROR_STREAM("Couldn't find transform from '"<<robot_frame<<"' to '"<<fixed_frame_<<"', retrying...");

        gm::Pose robot_pose = getPose(now, robot_frame);

        // Set up map dimensions
        nm::MapMetaData info;
        info.origin.position.x = robot_pose.position.x - local_grid_size_/2;
        info.origin.position.y = robot_pose.position.y - local_grid_size_/2;
        info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
        info.resolution = resolution_;
        info.width = local_grid_size_/resolution_;
        info.height = local_grid_size_/resolution_;

        fake_grid.info = info;
        map = gu::createCloudOverlay(fake_grid, fixed_frame_, 0.33, 10, 1);



        //Fill init gap
        //robot_pose.position.x += 0.12;
        //gu::addKnownFreePoint(&map, robot_pose.position, robot_outer_diameter/2, 10);

        current_iteration = 0;
    }

    void echoGrid(const ros::WallTimerEvent& scan) {
        nm::OccupancyGrid::ConstPtr grid = gu::getGrid(map);
        grid_pub_.publish(grid);

    }

    void executorState(const std_msgs::String &state) {
        if(state.data == "FORWARD" || state.data == "STOP") {
            isMapping = true;
        } else {
            isMapping = false;
        }
    }


    void mapIr(const sensors::SensorClouds &msg)
    {
        if (!isMapping)
            return;

        std::vector<sensor_msgs::PointCloud2> clouds = msg.point_clouds;
        std::vector<u_int8_t> hasEndpoint = msg.hasEndpoint;

        for(int pc = 0; pc < 6; pc++) {
            if (pc == 1) //skip rear sensor
                continue;

            string ir_frame = clouds[pc].header.frame_id;
            ros::Time stamp = clouds[pc].header.stamp;
            try {
                // We'll need the transform between sensor and fixed frames at the time when the scan was taken
                if (!tf_.waitForTransform(fixed_frame_, ir_frame, stamp, ros::Duration(0.1)))
                {
                    ROS_WARN_STREAM ("Timed out waiting for transform from " << ir_frame << " to "
                                     << fixed_frame_ << " at " << stamp.toSec());
                    return;
                }

                sm::PointCloud cloud_legacy;
                sm::convertPointCloud2ToPointCloud(clouds[pc], cloud_legacy);

                // Construct and save LocalizedCloud
                CloudPtr loc_cloud(new gu::LocalizedCloud());
                loc_cloud->hasEndpoint = hasEndpoint[pc];
                loc_cloud->cloud.points = cloud_legacy.points;
                loc_cloud->sensor_pose = getPose(stamp, ir_frame);
                loc_cloud->header.frame_id = fixed_frame_;

                Lock lock(mutex_);
                gu::addCloud(&map, loc_cloud, 30, hasEndpoint[pc]);

                //last_cloud_=loc_cloud;
                //clouds_.push_back(last_cloud_);
            }
            catch (tf::TransformException& e) {
              ROS_INFO ("Not saving scan due to tf lookup exception: %s",
                        e.what());
            }
        }
    }

    void mapMarkPoseExplored(const ros::WallTimerEvent& time)
    {
        ros::Time now = ros::Time::now();
        if (tf_.waitForTransform(fixed_frame_, robot_frame, now, ros::Duration(0.1))) {

            gu::addKnownFreePoint(&map, getPose(now, robot_frame).position, robot_outer_diameter/2, 10);
        }
    }

    /*void mapPointCloud(const vision_msgs::PreprocessedClouds &msg)
    {


        sm::PointCloud2 cloud = msg.others;
        string camera_frame = cloud.header.frame_id;
        ros::Time stamp = cloud.header.stamp;

        try
        {
            // We'll need the transform between sensor and fixed frames at the time when the scan was taken
            if (!tf_.waitForTransform(fixed_frame_, camera_frame, stamp, ros::Duration(0.1)))
            {
                ROS_WARN_STREAM ("Timed out waiting for transform from " << camera_frame << " to "
                                 << fixed_frame_ << " at " << stamp.toSec());
                return;
            }
        }
        catch (tf::TransformException& e)
        {
            ROS_INFO ("Not saving scan due to tf lookup exception: %s", e.what());
            return;
        }

        sm::PointCloud cloud_legacy;
        sm::convertPointCloud2ToPointCloud(cloud, cloud_legacy);

        sm::PointCloud cloud_legacy_transformed;
        tf_.transformPointCloud(camera_frame, cloud_legacy, cloud_legacy_transformed);

        // Construct and save LocalizedCloud
        CloudPtr loc_cloud(new gu::LocalizedCloud());
        loc_cloud->hasEndpoint = true;
        loc_cloud->cloud.points = cloud_legacy_transformed.points;
        loc_cloud->sensor_pose = getPose(stamp, camera_frame);
        loc_cloud->header.frame_id = fixed_frame_;

        Lock lock(mutex_);
        gu::addCloud(&map, loc_cloud, true);

        //last_cloud_=loc_cloud;
    }*/

    void mapCloud(const vision_msgs::PreprocessedClouds &msg)
    {
        if(!isMapping)
            return;

        sm::PointCloud2 cloudFloor = msg.plane;
        string camera_frame = cloudFloor.header.frame_id;
        ros::Time stamp = cloudFloor.header.stamp;

        sm::PointCloud2 cloudWall = msg.others;

        try
        {
            // We'll need the transform between sensor and fixed frames at the time when the scan was taken
            if (!tf_.waitForTransform(fixed_frame_, camera_frame, stamp, ros::Duration(0.1)))
            {
                ROS_WARN_STREAM ("Timed out waiting for transform from " << camera_frame << " to "
                                 << fixed_frame_ << " at " << stamp.toSec());
                return;
            }
        }
        catch (tf::TransformException& e)
        {
            ROS_INFO ("Not saving scan due to tf lookup exception: %s", e.what());
            return;
        }

        sm::PointCloud cloudFloor_legacy;
        sm::convertPointCloud2ToPointCloud(cloudFloor, cloudFloor_legacy);

        //sm::PointCloud cloudFloor_legacy_transformed;
        //tf_.transformPointCloud(fixed_frame_, cloudFloor_legacy, cloudFloor_legacy_transformed);

        for (int n=0; n < cloudFloor_legacy.points.size(); n++)
        {
            gm::Point32 pf_t = cloudFloor_legacy.points[n];
            gm::Point pf;
            pf.x = pf_t.x;
            pf.y = pf_t.y;
            pf.z = pf_t.z;

            gu::addFreePoint(&map, pf, 0.01);
        }

        sm::PointCloud cloudWall_legacy;
        sm::convertPointCloud2ToPointCloud(cloudWall, cloudWall_legacy);

        //sm::PointCloud cloudWall_legacy_transformed;
        //tf_.transformPointCloud(fixed_frame_, cloudWall_legacy, cloudWall_legacy_transformed);

        for (int n=0; n < cloudWall_legacy.points.size(); n++)
        {
            gm::Point32 pw_t = cloudWall_legacy.points[n];
            gm::Point pw;
            pw.x = pw_t.x;
            pw.y = pw_t.y;
            pw.z = pw_t.z;

            gu::addOccupiedPoint(&map, pw, 0.01);
        }
    }


    gm::Pose getPose(ros::Time stamp, string source_frame)
    {
        return getPose(stamp, source_frame, fixed_frame_);
    }
    gm::Pose getPose(ros::Time stamp, string source_frame, string target_frame)
    {
        tf::Stamped<tf::Pose> pose;
        tf_.transformPose(target_frame, tf::Stamped<tf::Pose> (identity, stamp, source_frame), pose);
        gm::Pose pose_msg;
        tf::poseTFToMsg(pose, pose_msg);
        return pose_msg;
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_node");
    MapNode map_node;
    map_node.run();
}

#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


tf::Pose identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));

using std::string;
namespace gm=geometry_msgs;
namespace gu=occupancy_grid_utils;

class Navigater
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_map;
    ros::Subscriber sub_clicked_point;
    ros::Publisher pub_twist;
    ros::Publisher pub_path;
    ros::Publisher pub_map;
    tf::TransformListener tfl;
    ros::WallTimer timer_pathfinding;
    string fixed_frame;
    string robot_frame;
    double robot_diameter;
    bool active;

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid inflated_map;

    geometry_msgs::Point goal;

public:
    Navigater()
        : nh("~")
        , fixed_frame("map")
        , robot_frame("robot")
        , active(false)
    {
        double rate = requireParameter<double>("pathfinding_rate");
        robot_diameter = requireParameter<double>("/base/diameter");

        sub_map = nh.subscribe("map_in", 1, &Navigater::callback_map, this);
        sub_clicked_point = nh.subscribe("/clicked_point", 1, &Navigater::callback_clicked_point, this);
        pub_twist = nh.advertise<geometry_msgs::Twist>("twist_out", 10);
        pub_path = nh.advertise<nav_msgs::Path>("planned_path", 1);
        pub_map = nh.advertise<nav_msgs::OccupancyGrid>("map_out", 1);
        timer_pathfinding = nh.createWallTimer(ros::WallDuration(1/rate), &Navigater::run_pathfinding, this);
    }

    void callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map = *msg;
        inflated_map = *gu::inflateObstacles(map, robot_diameter/2);
        inflated_map.header.frame_id = msg->header.frame_id;
        inflated_map.header.stamp = msg->header.stamp;
        pub_map.publish(inflated_map);
        //map = occupancy_grid_utils::createCloudOverlay(msg, fixed_frame, 0.1, 10, 1);
    }

    void callback_clicked_point(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        goal = msg->point;
        active = true;
    }

    void run_pathfinding(const ros::WallTimerEvent&)
    {
        if (!active)
            return;

        static gm::Point prev_goal;
        if (prev_goal.x != goal.x && prev_goal.y != goal.y)
        {
            prev_goal = goal;
            ROS_INFO("navigating to %-5.2f,%5.2f", goal.x, goal.y);
        }

        gu::Cell robot_cell = getCell(ros::Time(0), robot_frame, inflated_map);
        gu::Cell goal_cell = gu::pointCell(inflated_map.info, goal);
        boost::optional<gu::AStarResult> astar = gu::shortestPathAStar(inflated_map, robot_cell, goal_cell);

        //gu::shortestPathAStar(inflated_map, );
        //gu::singleSourceShortestPaths(inflated_map)

        if (astar)
        {
            gu::Path path = (*astar).first;
            std::vector<gm::Point> points = getPoints(inflated_map, path);
            publishPath(points);
        }
    }

    std::vector<gm::Point> getPoints(const nav_msgs::OccupancyGrid& grid, const gu::Path& path)
    {
        std::vector<gm::Point> points;

        for (int i=0; i<path.size(); i++)
        {
            gm::PointStamped ps, fixed_frame_ps;
            ps.header = grid.header;
            ps.point = gu::cellCenter(inflated_map.info, path[i]);
            tfl.transformPoint(fixed_frame, ps, fixed_frame_ps);
            points.push_back(fixed_frame_ps.point);
        }

        return points;
    }

    void publishPath(const std::vector<gm::Point>& path)
    {
        nav_msgs::Path msg_path;
        msg_path.header.frame_id = inflated_map.header.frame_id;
        msg_path.header.stamp = ros::Time::now();

        for (int i=0; i<path.size(); i++)
        {
            gm::Point curr = path[i];
            gm::Point next;
            if (i==path.size()-1)
                next = path[i];
            else
                next = path[i+1];

            gm::PoseStamped pose;
            pose.pose.position = curr;
            tf::Point currTF, nextTF;
            tf::pointMsgToTF(curr, currTF);
            tf::pointMsgToTF(next, nextTF);
            tf::Quaternion q = tf::shortestArcQuatNormalize2(currTF, nextTF);
            tf::quaternionTFToMsg(q, pose.pose.orientation);
            pose.header.frame_id = inflated_map.header.frame_id;
            pose.header.stamp = ros::Time(0); //TODO: should maybe be the approximate time in the future
            msg_path.poses.push_back(pose);
        }

        pub_path.publish(msg_path);
    }


    template<typename T>
    T requireParameter(const string& name)
    {
        if (!nh.hasParam(name))
        {
            ROS_FATAL_STREAM("couldn't find parameter "<<nh.resolveName(name));
            ros::shutdown();
            exit(0);
        }
        T value;
        nh.getParam(name, value);
        ROS_INFO_STREAM("loaded parameter "<<nh.resolveName(name)<<": "<<value);
        return value;
    }

    gu::Cell getCell(const ros::Time& stamp, string source_frame, nav_msgs::OccupancyGrid map)
    {
        gm::Pose pose = getPose(stamp, source_frame, map.header.frame_id);
        return gu::pointCell(map.info, pose.position);
    }
    gm::Pose getPose(const ros::Time& stamp, string source_frame)
    {
        return getPose(stamp, source_frame, fixed_frame);
    }
    gm::Pose getPose(const ros::Time& stamp, string source_frame, string target_frame)
    {
        tf::Stamped<tf::Pose> pose;
        tfl.transformPose(target_frame, tf::Stamped<tf::Pose> (identity, stamp, source_frame), pose);
        gm::Pose pose_msg;
        tf::poseTFToMsg(pose, pose_msg);
        return pose_msg;
    }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "navigater");
    Navigater navigation_node;
    navigation_node.run();
}

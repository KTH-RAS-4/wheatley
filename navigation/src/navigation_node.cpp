#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <boost/function.hpp>
#include <wheatley_common/common.h>
#include <wheatley_common/angles.h>
#define _USE_MATH_DEFINES
#include <cmath>


using std::string;
namespace gm=geometry_msgs;
namespace gu=occupancy_grid_utils;

namespace wheatley
{
    class Navigator : NiceBaseClass
    {
    private:
        ros::Subscriber sub_map;
        ros::Subscriber sub_clicked_point;
        ros::Subscriber sub_pose;

        ros::Publisher pub_path;
        ros::Publisher pub_map;
        tf::TransformListener tfl;
        ros::WallTimer timer_pathfinding;
        const string fixed_frame;
        const string robot_frame;
        const double robot_diameter;
        bool has_goal;

        nav_msgs::OccupancyGrid map;
        nav_msgs::OccupancyGrid inflated_map;
        gm::Pose pose;
        gm::PointStamped home;

        gm::Point goal;
        int phase;
        ros::Time last_input;

        double timeout;
        ros::Time timeWhenLeftHome;
        double phaseTimeout;

    public:
        Navigator()
            : fixed_frame("map")
            , robot_frame("robot")
            , has_goal(false)
            , last_input(-1000)
            , robot_diameter(requireParameter<double>("/base/diameter"))
            , phase(requireParameter<int>("/phase"))
        {
            double rate = requireParameter<double>("pathfinding_rate");
            double timeout = requireParameter<double>("timeout");


            sub_map = nh.subscribe("map_in", 1, &Navigator::callback_map, this);
            sub_clicked_point = nh.subscribe("/clicked_point", 1, &Navigator::callback_clicked_point, this);
            sub_pose = nh.subscribe("pose", 1, &Navigator::callback_pose, this);
            pub_path = nh.advertise<nav_msgs::Path>("planned_path", 1);
            pub_map = nh.advertise<nav_msgs::OccupancyGrid>("map_out", 1);
            timer_pathfinding = nh.createWallTimer(ros::WallDuration(1/rate), &Navigator::callback_timer, this);
            timeWhenLeftHome = ros::Time::now();
            timeWhenLeftHome = ros::Time::now();

            if (phase == 1)
                phaseTimeout = 270;
            else if (phase == 2)
                phaseTimeout = 150;
        }

        signed char occupancyFunction(float distance)
        {
            const float min = robot_diameter/2;
            const float max = robot_diameter/2 + 0.1;

            if (distance <= min)
                return gu::OCCUPIED;
            else if (distance <= max)
            {
                //ROS_INFO("%f %d", distance, (int)(std::exp(-(distance-min)/(max-min)) * (gu::OCCUPIED-1)));
                //return std::exp(-(distance-min)/(max-min)) * (gu::OCCUPIED-1);
                return 0.7*(1-(distance-min)/(max-min))*(gu::OCCUPIED-1);
            }
            else
                return gu::UNKNOWN;
        }

        void callback_pose(const nav_msgs::Odometry::ConstPtr& msg)
        {
            static bool firstPose = true;
            if (firstPose)
            {
                firstPose = false;
                home.point.x = msg->pose.pose.position.x;
                home.point.y = msg->pose.pose.position.y;
            }
        }

        void callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
        {
            ROS_INFO_ONCE("got first map");
            map = *msg;
            inflated_map = gu::inflateObstacles(map,
                                                boost::bind(&Navigator::occupancyFunction, this, _1),
                                                true);
            inflated_map.header.frame_id = msg->header.frame_id;
            inflated_map.header.stamp = msg->header.stamp;
            pub_map.publish(inflated_map);
            run_pathfinding();
        }

        void callback_clicked_point(const geometry_msgs::PointStamped::ConstPtr& msg)
        {
            if (phase == 0)
            {
                goal = msg->point;
                has_goal = true;
                run_pathfinding();
            }
        }

        void callback_timer(const ros::WallTimerEvent&)
        {
            run_pathfinding();
        }

        void run_pathfinding()
        {
            static bool phaseTimedOut = true;
            /*if ((ros::Time::now()-timeWhenLeftHome).toSec() > phaseTimeout)
            {
                if (!phaseTimedOut)
                {
                    phaseTimedOut = true;
                    goal = home.point;
                    has_goal = true;
                    phase = 0;
                    ROS_INFO("Time is running out, going home!");
                }
            }*/
            static bool timedOut = true;
            /*if ((ros::Time::now()-last_input).toSec() > timeout)
            {
                if (!timedOut)
                {
                    timedOut = true;
                    goal = home.point;
                    has_goal = true;
                    ROS_INFO("timeout, going home!");
                }
            }*/

            if (phase != 1 && !has_goal)
                return;

            if (map.header.frame_id == "")
            {
                ROS_ERROR_STREAM_ONCE("no map received yet, will pathfind when map is provided");
                return;
            }

            if (!tfl.waitForTransform(robot_frame, fixed_frame, ros::Time(0), ros::Duration(0.2)))
            {
                ROS_ERROR_STREAM("timeout waiting for transform from "<<fixed_frame<<" to "<<robot_frame);
                return;
            }

            if (phase != 1)
            {
                static gm::Point prev_goal;
                if (prev_goal.x != goal.x && prev_goal.y != goal.y)
                {
                    prev_goal = goal;
                    ROS_INFO("navigating to %5.2f,%-5.2f", goal.x, goal.y);
                }
            }

            tf::StampedTransform robot_transform;
            tfl.lookupTransform(fixed_frame, robot_frame, ros::Time(0), robot_transform);
            angles::StraightAngle currDirection = angles::StraightAngle::getClosest(tf::getYaw(robot_transform.getRotation()));

            gu::Cell robot_cell = getCell(ros::Time(0), robot_frame, inflated_map);
            boost::optional<gu::Cell> free_robot_cell = gu::closestFree(inflated_map, robot_cell, 0.3);
            gu::Cell goal_cell = gu::pointCell(inflated_map.info, goal);
            boost::optional<gu::Cell> free_goal_cell = gu::closestFree(inflated_map, goal_cell, 0.3);

            if (phase == 1)
                free_goal_cell = free_robot_cell; //ugly fix :D

            if (!free_robot_cell || !free_goal_cell)
            {
                ROS_ERROR("unable to find any close by free point");
                return;
            }

            boost::optional<gu::AStarResult> astar = gu::shortestPathAStar(inflated_map, *free_robot_cell, *free_goal_cell, phase == 1, currDirection);

            if (!astar)
            {
                ROS_ERROR("no path to goal");
                return;
            }

            gu::Path path = (*astar).first;
            std::vector<gm::Point> points = getPoints(inflated_map, path);

            publishPath(points);
            last_input = ros::Time::now();
            timedOut = false;
            phaseTimedOut = false;

        }

        void publishPath(const std::vector<gm::Point>& path)
        {
            nav_msgs::Path msg_path;
            msg_path.header.frame_id = inflated_map.header.frame_id;
            msg_path.header.stamp = ros::Time::now();

            for (int i=0; i<path.size(); i++)
            {
                double angle;
                if (i<path.size()-1)
                    angle = angles::angleOf(path[i], path[i+1]);
                else
                    angle = angles::angleOf(path[i-1], path[i]);

                gm::PoseStamped pose;
                pose.pose.position = path[i];
                tf::Quaternion q = tf::createQuaternionFromYaw(angle);
                tf::quaternionTFToMsg(q, pose.pose.orientation);
                pose.header.frame_id = inflated_map.header.frame_id;
                pose.header.stamp = ros::Time(0); //TODO: should maybe be the approximate time in the future
                msg_path.poses.push_back(pose);
            }

            pub_path.publish(msg_path);
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

        gu::Cell getCell(const ros::Time& stamp, string source_frame, nav_msgs::OccupancyGrid map)
        {
            gm::Pose pose = getPose(tfl, stamp, source_frame, map.header.frame_id);
            return gu::pointCell(map.info, pose.position);
        }

        void run()
        {
            ros::spin();
        }
    };
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "navigator");
    wheatley::Navigator navigation_node;
    navigation_node.run();
}

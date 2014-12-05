#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
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
        ros::NodeHandle nh;
        ros::Subscriber sub_map;
        ros::Subscriber sub_clicked_point;
        ros::Subscriber sub_executor_state;
        ros::Publisher pub_executor_order;
        ros::Publisher pub_path;
        ros::Publisher pub_map;
        tf::TransformListener tfl;
        ros::WallTimer timer_pathfinding;
        const string fixed_frame;
        const string robot_frame;
        const double robot_outer_diameter;
        bool active;
        bool executor_ready;
        string prev_order;

        nav_msgs::OccupancyGrid map;
        nav_msgs::OccupancyGrid inflated_map;

        geometry_msgs::Point goal;

    public:
        Navigator()
            : nh("~")
            , fixed_frame("map")
            , robot_frame("robot")
            , active(false)
            , executor_ready(false)
            , robot_outer_diameter(requireParameter<double>("/base/outer_diameter"))
        {
            double rate = requireParameter<double>("pathfinding_rate");

            sub_map = nh.subscribe("map_in", 1, &Navigator::callback_map, this);
            sub_clicked_point = nh.subscribe("/clicked_point", 1, &Navigator::callback_clicked_point, this);
            sub_executor_state = nh.subscribe("executor_state", 1, &Navigator::callback_executor_state, this);
            pub_executor_order = nh.advertise<std_msgs::String>("executor_order", 1, true);
            pub_path = nh.advertise<nav_msgs::Path>("planned_path", 1);
            pub_map = nh.advertise<nav_msgs::OccupancyGrid>("map_out", 1);
            timer_pathfinding = nh.createWallTimer(ros::WallDuration(1/rate), &Navigator::callback_timer, this);
        }

        void callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
        {
            ROS_INFO_ONCE("got first map");
            map = *msg;
            inflated_map = gu::inflateObstacles(map,
                                                robot_outer_diameter/2,
                                                robot_outer_diameter/2 + 0.1,
                                                true);
            inflated_map.header.frame_id = msg->header.frame_id;
            inflated_map.header.stamp = msg->header.stamp;
            pub_map.publish(inflated_map);
            run_pathfinding();
        }

        void callback_clicked_point(const geometry_msgs::PointStamped::ConstPtr& msg)
        {
            goal = msg->point;
            active = true;
            prev_order.clear();
            run_pathfinding();
        }

        void callback_executor_state(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO_STREAM_ONCE("got first state from executor: " << msg->data);
            if (msg->data == "STOP")
                prev_order.clear();
            if (msg->data == "STOP" || msg->data == "FORWARD")
                executor_ready = true;
            run_pathfinding();
        }

        void callback_timer(const ros::WallTimerEvent&)
        {
            run_pathfinding();
        }

        void run_pathfinding()
        {
            if (!active || !executor_ready) //don't bother if the executor is turning
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

            static gm::Point prev_goal;
            if (prev_goal.x != goal.x && prev_goal.y != goal.y)
            {
                prev_goal = goal;
                ROS_INFO("navigating to %5.2f,%-5.2f", goal.x, goal.y);
            }

            tf::StampedTransform robot_transform;
            tfl.lookupTransform(fixed_frame, robot_frame, ros::Time(0), robot_transform);
            angles::StraightAngle currDirection = angles::StraightAngle::getClosest(tf::getYaw(robot_transform.getRotation()));

            gu::Cell robot_cell = getCell(ros::Time(0), robot_frame, inflated_map);
            boost::optional<gu::Cell> closestFree = gu::closestFree(inflated_map, robot_cell, 0.3);

            if (!closestFree)
            {
                ROS_ERROR("unable to find any close by free point");
                return;
            }

            robot_cell = *closestFree;

            gu::Cell goal_cell = gu::pointCell(inflated_map.info, goal);

            boost::optional<gu::AStarResult> astar = gu::shortestPathAStar(inflated_map, robot_cell, goal_cell, currDirection);

            //gu::singleSourceShortestPaths(inflated_map)

            if (!astar)
            {
                ROS_ERROR("unable to find any path to goal");
                return;
            }

            gu::Path path = (*astar).first;
            std::vector<gm::Point> points = getPoints(inflated_map, path);

            if (points.size() < 4)
            {
                publishPath(std::vector<gm::Point>());
                active = false;
                publishOrder("STOP");
            }
            else
            {
                publishPath(points);

                angles::StraightAngle nextDirection = angles::StraightAngle::getClosest(points[0], points[1]);
                double diff = angles::shortest_angular_distance(currDirection.angle(), nextDirection.angle());

                if (diff > 0)
                    publishOrder("LEFT");
                else if (diff < 0)
                    publishOrder("RIGHT");
                else
                    publishOrder("FORWARD");
            }
        }

        void publishOrder(string order)
        {
            if (executor_ready && prev_order != order)
            {
                prev_order = order;

                if (order == "LEFT" || order == "RIGHT")
                    executor_ready = false;

                std_msgs::String msg_order;
                msg_order.data = order;
                pub_executor_order.publish(msg_order);
            }
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
                tf::Quaternion q = tf::createQuaternionFromYaw(angles::angleOf(curr, next));
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

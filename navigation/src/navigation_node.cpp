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
#define _USE_MATH_DEFINES
#include <cmath>


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
    ros::Subscriber sub_executor_state;
    ros::Publisher pub_executor_order;
    ros::Publisher pub_path;
    ros::Publisher pub_map;
    tf::TransformListener tfl;
    ros::WallTimer timer_pathfinding;
    const string fixed_frame;
    const string robot_frame;
    const double robot_diameter;
    bool active;
    bool executor_ready;

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid inflated_map;

    geometry_msgs::Point goal;

public:
    Navigater()
        : nh("~")
        , fixed_frame("map")
        , robot_frame("robot")
        , active(false)
        , executor_ready(false)
        , robot_diameter(requireParameter<double>("/base/diameter"))
    {
        double rate = requireParameter<double>("pathfinding_rate");

        sub_map = nh.subscribe("map_in", 1, &Navigater::callback_map, this);
        sub_clicked_point = nh.subscribe("/clicked_point", 1, &Navigater::callback_clicked_point, this);
        sub_executor_state = nh.subscribe("executor_state", 1, &Navigater::callback_executor_state, this);
        pub_executor_order = nh.advertise<std_msgs::String>("executor_order", 1, true);
        pub_path = nh.advertise<nav_msgs::Path>("planned_path", 1);
        pub_map = nh.advertise<nav_msgs::OccupancyGrid>("map_out", 1);
        timer_pathfinding = nh.createWallTimer(ros::WallDuration(1/rate), &Navigater::callback_timer, this);
    }

    void callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map = *msg;
        inflated_map = *gu::inflateObstacles(map, robot_diameter/2);
        inflated_map.header.frame_id = msg->header.frame_id;
        inflated_map.header.stamp = msg->header.stamp;
        pub_map.publish(inflated_map);
        //map = occupancy_grid_utils::createCloudOverlay(msg, fixed_frame, 0.1, 10, 1);
        run_pathfinding();
    }

    void callback_clicked_point(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        goal = msg->point;
        active = true;
        run_pathfinding();
    }

    void callback_executor_state(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO_STREAM_ONCE("got first state from executor: " << msg->data);
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

        gu::Cell robot_cell = getCell(ros::Time(0), robot_frame, inflated_map);
        gu::Cell goal_cell = gu::pointCell(inflated_map.info, goal);
        boost::optional<gu::AStarResult> astar = gu::shortestPathAStar(inflated_map, robot_cell, goal_cell);

        //gu::singleSourceShortestPaths(inflated_map)

        if (astar)
        {
            gu::Path path = (*astar).first;
            std::vector<gm::Point> points = getPoints(inflated_map, path);

            if (points.size() < 2)
            {
                publishPath(std::vector<gm::Point>());
                active = false;
                publishOrder("STOP");
            }
            else
            {
                publishPath(points);

                tf::StampedTransform robot_transform;
                tfl.lookupTransform(fixed_frame, robot_frame, ros::Time(0), robot_transform);

                double nextDirection = closestRightAngle(angle(points[0], points[1]));
                double currentDirection = closestRightAngle(tf::getYaw(robot_transform.getRotation()));
                double diff = angleDiff(nextDirection, currentDirection);

                if (diff > 0)
                    publishOrder("LEFT");
                else if (diff < 0)
                    publishOrder("RIGHT");
                else
                    publishOrder("FORWARD");
            }
        }
    }

    void publishOrder(string order)
    {
        static string prev_order;
        if (prev_order != order)
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
            tf::Quaternion q = tf::createQuaternionFromYaw(angle(curr, next));
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

    tf::Point toTF(gm::Point p)
    {
        tf::Point p_tf;
        tf::pointMsgToTF(p, p_tf);
        return p_tf;
    }

    double angleDiff(double a, double b)
    {
        return constrainAngle(a-b);
    }

    /* returns the closest of the angles [0, PI/2, PI, PI/2*3] */
    double closestRightAngle(double angle)
    {
        for (double g = 0; g < M_PI*5/3; g += M_PI/2)
        {
            if (std::abs(angleDiff(g, angle)) <= M_PI/4)
                return g;
        }
        throw std::runtime_error("hmm, couldn't find closest angle");
    }

    /* normalize a given angle to [0, 2*pi) */
    double constrainAnglePos(double angle)
    {
        angle = fmod(angle, M_PI*2);
        if (angle < 0)
            angle += M_PI*2;
        return angle;
    }

    /* normalize a given angle to (-pi, pi] */
    double constrainAngle(double angle)
    {
        angle = fmod(angle+M_PI, M_PI*2);
        if (angle <= 0)
            angle += M_PI*2;
        return angle-M_PI;
    }

    double angle(gm::Point from, gm::Point to)
    {
        return std::atan2(to.y-from.y, to.x-from.x);
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

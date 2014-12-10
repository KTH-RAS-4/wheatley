#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <wheatley_common/common.h>
#include <wheatley_common/angles.h>
#define _USE_MATH_DEFINES
#include <cmath>


using std::string;

namespace wheatley
{
    class PathFollower : NiceBaseClass
    {
    private:
        ros::Publisher  pub_executor_order;
        ros::Subscriber sub_executor_state;
        ros::Subscriber sub_path;
        ros::Subscriber sub_pose;

        const string fixed_frame;
        const string robot_frame;
        const int phase;
        const double stoppingDistance;
        const double maxProjectionDistance;

        std::vector<tf::Pose> path;
        tf::Pose pose;

        bool executor_ready;
        string prev_order;

    public:
        PathFollower()
            : fixed_frame("map")
            , robot_frame("robot")
            , executor_ready(false)
            , phase(requireParameter<int>("/phase"))
            , stoppingDistance(requireParameter<double>("stopping_distance"))
            , maxProjectionDistance(requireParameter<double>("max_projection_distance"))
        {
            pub_executor_order = nh.advertise<std_msgs::String>("executor_order", 1, true);
            sub_executor_state = nh.subscribe("executor_state", 1, &PathFollower::callback_executor_state, this);
            sub_pose = nh.subscribe("pose", 1, &PathFollower::callback_pose, this);
            sub_path = nh.subscribe("path", 1, &PathFollower::callback_path, this);
        }

        void callback_path(const nav_msgs::Path::ConstPtr& msg)
        {
            ROS_INFO_STREAM_ONCE("got first path");
            path.clear();
            for (int i=0; i < msg->poses.size(); i++)
            {
                tf::Pose pose;
                geometry_msgs::PoseStamped s;
                tf::poseMsgToTF(msg->poses[i].pose, pose);
                path.push_back(pose);
            }

            run_change_direction();
        }

        void callback_pose(const nav_msgs::Odometry::ConstPtr& msg)
        {
            tf::poseMsgToTF(msg->pose.pose, pose);

            //set theta to the closest
            tf::Quaternion q;
            q.setRPY(0,0, yaw(pose.getRotation()).angle());
            pose.setRotation(q);

            run_change_direction();
        }

        void callback_executor_state(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO_STREAM_ONCE("got first state from executor: " << msg->data);
            if (msg->data == "STOP")
                prev_order.clear();
            if (msg->data == "STOP" || msg->data == "FORWARD")
                executor_ready = true;

            run_change_direction();
        }

       void run_change_direction()
        {
            if (phase != 0 && !executor_ready) //phase 0 is for debugging
                return;

            if (path.empty())
                return;

            tf::Pose goal = path.back();

            if (yaw(pose.getRotation()) == yaw(goal.getRotation()) &&
                    pose.getOrigin().distance(goal.getOrigin()) <= stoppingDistance)
            {
                publishOrder("STOP");
                path.clear();
            }
            else
            {
                boost::optional<tf::Pose> closest = getClosest(path, pose, maxProjectionDistance);
                if (closest)
                {
                    double diff = angleDiff(pose, *closest);

                    if (diff > 0)
                        publishOrder("LEFT");
                    else if (diff < 0)
                        publishOrder("RIGHT");
                    else
                        publishOrder("FORWARD");
                }
                else
                    publishOrder("STOP");
            }
        }

        void publishOrder(string order)
        {
            if ((executor_ready || phase == 0) && prev_order != order)
            {
                prev_order = order;

                if (order == "LEFT" || order == "RIGHT" || order == "BACK")
                    executor_ready = false;

                std_msgs::String msg_order;
                msg_order.data = order;
                pub_executor_order.publish(msg_order);
            }
        }

        static angles::StraightAngle yaw(tf::Quaternion q)
        {
            return angles::StraightAngle::getClosest(tf::getYaw(q));
        }

        static double angleDiff(tf::Pose a, tf::Pose b)
        {
            return angles::shortest_angular_distance(yaw(a.getRotation()).angle(),
                                                     yaw(b.getRotation()).angle());
        }

        static boost::optional<tf::Pose> getClosest(std::vector<tf::Pose> path, tf::Pose pose, double maxDistance)
        {
            double best = maxDistance;
            boost::optional<tf::Pose> closest;

            for (int i=0; i<path.size(); i++)
            {
                double anglediff = fabs(angleDiff(pose, path[i])/M_PI);
                //anglediff is
                // 0 for 0 degrees wrong,
                // 1 for 90 degrees wrong
                // 2 for 180 degrees wrong

                double distance = pose.getOrigin().distance(path[i].getOrigin());
                double dist = distance * (anglediff+1) + anglediff*0.02; //great way of measuring similarity :D
                if (best >= dist)
                {
                    best = dist;
                    closest = path[i];
                }
            }

            return closest;
        }



        void run()
        {
            ros::spin();
        }
    };
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pathfollower");
    wheatley::PathFollower path_following_node;
    path_following_node.run();
}

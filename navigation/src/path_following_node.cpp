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
#include <sensors/Distance.h>
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
        ros::Subscriber sub_ir_distances;

        const string fixed_frame;
        const string robot_frame;
        const int phase;
        const double stoppingDistance;
        const double stopForwardDistance;
        const double maxProjectionDistance;
        const double maxSkipTurnGoTurnBackDistance;

        std::vector<tf::Pose> path;
        sensors::Distance distance;
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
            , stopForwardDistance(requireParameter<double>("stop_forward_distance"))
            , maxProjectionDistance(requireParameter<double>("max_projection_distance"))
            , maxSkipTurnGoTurnBackDistance(requireParameter<double>("max_skip_turn_go_turn_back_distance"))
        {
            pub_executor_order = nh.advertise<std_msgs::String>("executor_order", 1, true);
            sub_executor_state = nh.subscribe("executor_state", 1, &PathFollower::callback_executor_state, this);
            sub_pose = nh.subscribe("pose", 1, &PathFollower::callback_pose, this);
            sub_path = nh.subscribe("path", 1, &PathFollower::callback_path, this);
            sub_ir_distances = nh.subscribe ("/sensors/ir/distances", 1, &PathFollower::callback_ir_distances, this);
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

        void callback_ir_distances(const sensors::Distance::ConstPtr& msg)
        {
            ROS_INFO_STREAM_ONCE("got first ir");
            distance = *msg;
        }

        void callback_pose(const nav_msgs::Odometry::ConstPtr& msg)
        {
            tf::poseMsgToTF(msg->pose.pose, pose);

            //set theta to the closest
            tf::Quaternion q;
            q.setRPY(0,0, yaw(pose).angle());
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

            if (yaw(pose) == yaw(goal) &&
                    pose.getOrigin().distance(goal.getOrigin()) <= stoppingDistance)
            {
                publishOrder("STOP");
                path.clear();
            }
            else
            {
                boost::optional<int> currIndex = getClosestIndex(path, pose, maxProjectionDistance);
                if (currIndex)
                {
                    int currTurn = turnDiff(pose, path[*currIndex]);

                    int nextTurnIndex = -1;
                    for (int i=*currIndex+1; i<path.size(); i++)
                    {
                        if (turnDiff(path[i-1], path[i]) != 0)
                        {
                            nextTurnIndex = i;
                            break;
                        }
                    }

                    int nextTurn = 0;
                    if (nextTurnIndex != -1)
                    {
                        nextTurn = turnDiff(path[nextTurnIndex-1], path[nextTurnIndex]);
                        double distanceToNextTurn = path[*currIndex].getOrigin().distance(path[nextTurnIndex].getOrigin());

                        //if turning 180 and the next turn is to the left, turn back to the right instead
                        if (currTurn == 2 && nextTurn == 1)
                            currTurn = -2;

                        //skip turning left just to turn right again shortly
                        if (distanceToNextTurn <= maxSkipTurnGoTurnBackDistance && currTurn == -nextTurn)
                            currTurn = 0;
                    }

                    if (currTurn == 0 && prev_order == "FORWARD" && distance.front < stopForwardDistance)
                    {
                        if (nextTurn != 0)
                            currTurn = nextTurn;
                        else
                            currTurn = 2;
                    }


                    if (currTurn > 0)
                        publishOrder("LEFT");
                    else if (currTurn < 0)
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

        static angles::StraightAngle yaw(tf::Pose p)
        {
            return angles::StraightAngle::getClosest(tf::getYaw(p.getRotation()));
        }

        static int turnDiff(tf::Pose a, tf::Pose b)
        {
            //-1 for -90 degrees
            // 0 for 0 degrees
            // 1 for 90 degrees
            // 2 for 180 degrees
            return (yaw(b).index()-yaw(a).index()+2)%4 - 2;
        }

        static boost::optional<int> getClosestIndex(std::vector<tf::Pose> path, tf::Pose pose, double maxDistance)
        {
            double best = maxDistance;
            boost::optional<int> closest;

            for (int i=0; i<path.size(); i++)
            {
                int turndiff = abs(turnDiff(pose, path[i]));

                double distance = pose.getOrigin().distance(path[i].getOrigin());
                double dist = distance + turndiff*0.05; //even greater way of measuring similarity :D
                if (best >= dist)
                {
                    best = dist;
                    closest = i;
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

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
#include <boost/circular_buffer.hpp>
#define _USE_MATH_DEFINES
#include <cmath>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
    return value < low ? low : (value > high ? high : value);
}

enum state { FOLLOW, ALIGN, FIND_ALIGNMENT };

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
    ros::Publisher pub_motor_twist;
    ros::Publisher pub_wall_twist;

    const string fixed_frame;
    const string robot_frame;
    const int phase;
    const double stoppingDistance;
    const double minStopForwardDistance;
    const double maxStopForwardDistance;
    const double maxProjectionDistance;
    const double maxSkipTurnGoTurnBackDistance;
    const double prevPoseMemory;
    const double prevPoseSaveRate;
    const double prevPoseAvoidDistance;
    const double newOrderCooldown;
    const double maxSkipForwardAndTurn;
    const double belowWhichSpeedClassifiesAsSlow;
    const double stopBeforeTurningDistance;
    const int time_until_path;

    bool poseCorrL;
    bool poseCorrR;
    int count;
    double leftDiff [10];
    double rightDiff[10];
    double lR;
    double lL;
    double avgLeftDiff;
    double avgRightDiff;
    double maxWallAlignDistance;
    double maxWallAlignAngle;
    double minTurnTwist;

    boost::circular_buffer<tf::Stamped<tf::Pose> > prev_poses;
    std::vector<tf::Pose> path;
    sensors::Distance distance;
    tf::Pose pose;

    bool executor_ready;
    string prev_order;

    ros::Publisher pub_pose_correction;
    ros::Time start_time;
    ros::Rate loop_rate;
    double desiredTheta;
    double theta;
    bool wallBrain;
    string state_str;

public:
    PathFollower()
        : fixed_frame("map")
        , robot_frame("robot")
        , executor_ready(false)
        , phase(requireParameter<int>("/phase"))
        , prevPoseMemory(requireParameter<double>("prev_pose_memory"))
        , prevPoseSaveRate(requireParameter<double>("prev_pose_save_rate"))
        , prevPoseAvoidDistance(requireParameter<double>("prev_pose_avoid_distance"))
        , stoppingDistance(requireParameter<double>("stopping_distance"))
        , minStopForwardDistance(requireParameter<double>("min_stop_forward_distance"))
        , maxStopForwardDistance(requireParameter<double>("max_stop_forward_distance"))
        , maxProjectionDistance(requireParameter<double>("max_projection_distance"))
        , maxSkipTurnGoTurnBackDistance(requireParameter<double>("max_skip_turn_go_turn_back_distance"))
        , newOrderCooldown(requireParameter<double>("new_order_cooldown"))
        , maxSkipForwardAndTurn(requireParameter<double>("max_skip_forward_and_turn"))
        , belowWhichSpeedClassifiesAsSlow(requireParameter<double>("below_which_speed_classifies_as_slow"))
        , stopBeforeTurningDistance(requireParameter<double>("stop_before_turning_distance"))
        , prev_poses(ceil(prevPoseMemory*prevPoseSaveRate))
        , loop_rate(10)
        , time_until_path(requireParameter<double>("time_until_path"))
        , start_time(ros::Time::now())
        , theta(0)
        , desiredTheta(0)
        , wallBrain(true)
        , poseCorrL(false)
        , poseCorrR(false)
        , count(0)
        , maxWallAlignDistance(requireParameter<double>("max_wall_align_distance"))
        , maxWallAlignAngle(requireParameter<double>("max_wall_align_angle")*M_PI/180)
        , minTurnTwist(requireParameter<double>("minTurnTwist"))
    {
        pub_pose_correction = nh.advertise<nav_msgs::Odometry> ("pose_correction", 1);
        pub_executor_order = nh.advertise<std_msgs::String>("executor_order", 1, true);
        sub_executor_state = nh.subscribe("executor_state", 1, &PathFollower::callback_executor_state, this);
        sub_pose = nh.subscribe("pose", 1, &PathFollower::callback_pose, this);
        sub_path = nh.subscribe("path", 1, &PathFollower::callback_path, this);
        sub_ir_distances = nh.subscribe ("/sensors/ir/distances", 1, &PathFollower::callback_ir_distances, this);
        pub_motor_twist = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
        pub_wall_twist = nh.advertise<geometry_msgs::Twist>("/wall_avoider/twist", 1000);
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

        if (poseCorrL)
        {
            leftDiff[count++] = atan(-(distance.left_front-distance.left_rear)/lL);

            double sumL;
            for(int i = 0; i < count; i++)
            {
               sumL += leftDiff[i];
            }
           avgLeftDiff = sumL / (double) count;
        }
        else if (poseCorrR)
        {
            rightDiff[count++] = atan((distance.right_front-distance.right_rear)/lR);

            double sumR;
            for(int i = 0; i < count; i++)
            {
               sumR += rightDiff[i];
            }
            avgRightDiff = sumR / (double) count;
        }
    }

    void callback_pose(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf::poseMsgToTF(msg->pose.pose, pose);
        theta = tf::getYaw(msg->pose.pose.orientation);
        //set theta to the closest
        //tf::Quaternion q;
        //q.setRPY(0,0, yaw(pose).angle());
        //pose.setRotation(q);

        static ros::Time prev_save_time(-1000);
        if ((msg->header.stamp - prev_save_time).toSec() > 1/prevPoseSaveRate)
        {
            prev_save_time = msg->header.stamp;
            prev_poses.push_back(tf::Stamped<tf::Pose>(pose, msg->header.stamp, msg->header.frame_id));
        }
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
        if(wallBrain)
            return;

        if (phase != 0 && !executor_ready) //phase 0 is for debugging
            return;

        if (path.empty())
        {
            ROS_INFO_ONCE("no path yet");
            return;
        }

        tf::Pose goal = path.back();

        bool exploringNewFrontiers = true;
        tf::Point forwardPoint = pose * tf::Vector3(prevPoseAvoidDistance*1.5, 0, 0);
        for (int i=0; i<prev_poses.size(); i++)
        {
            if (forwardPoint.distance2(prev_poses[i].getOrigin()) < pow(prevPoseAvoidDistance,2))
            {
                exploringNewFrontiers = false;
                break;
            }
        }

        /*if (!exploringNewFrontiers &&
                    yaw(pose) == yaw(goal) &&
                    pose.getOrigin().distance(goal.getOrigin()) <= stoppingDistance)
            {
                publishOrder("STOP");
                path.clear();
            }
            else*/
        {
            boost::optional<int> currIndex = getClosestIndex(path, pose, maxProjectionDistance);
            if (currIndex)
            {
                tf::Stamped<tf::Pose> pc = prev_poses[prev_poses.size() - 1];
                tf::Stamped<tf::Pose> pp = prev_poses[prev_poses.size() - 2];
                double speed = pc.getOrigin().distance(pp.getOrigin()) / (pc.stamp_ - pp.stamp_).toSec();

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

                    if (distanceToNextTurn <= maxSkipForwardAndTurn && speed < belowWhichSpeedClassifiesAsSlow && currTurn == 0 && nextTurn != 0)
                        currTurn = nextTurn;

                    if (distanceToNextTurn <= stopBeforeTurningDistance && speed > belowWhichSpeedClassifiesAsSlow && currTurn == 0)
                        currTurn = nextTurn;
                }

                if (prev_order == "FORWARD" && distance.front < (speed > belowWhichSpeedClassifiesAsSlow? maxStopForwardDistance : minStopForwardDistance))
                {
                    if (currTurn == 0)
                    {
                        if (nextTurn != 0)
                            currTurn = nextTurn;
                        else
                            currTurn = 2;
                    }
                }
                else if (prev_order == "FORWARD" && exploringNewFrontiers)
                    currTurn = 0; //don't interrupt a perfectly good forward

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
        static ros::Time prev_order_time(-1000);
        ros::Time now = ros::Time::now();

        if (prev_order == "FORWARD" && (order == "LEFT" || order == "RIGHT" || order == "BACK"))
            order = "STOP";

        if (order == "STOP" || (now-prev_order_time).toSec() > newOrderCooldown)
        {
            if ((executor_ready || phase == 0) && prev_order != order)
            {
                prev_order = order;
                prev_order_time = now;

                if (order == "LEFT" || order == "RIGHT" || order == "BACK")
                    executor_ready = false;

                std_msgs::String msg_order;
                msg_order.data = order;
                pub_executor_order.publish(msg_order);
            }
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
            double dist = distance + turndiff*0.03; //even greater way of measuring similarity :D
            if (best >= dist)
            {
                best = dist;
                closest = i;
            }
        }

        return closest;
    }


    void publishOdometry(ros::Time now, double diff)
    {
      tf::Quaternion q;
      q.setRPY(0, 0, desiredTheta + diff);

      //publish pose
      nav_msgs::Odometry pose;
      pose.header.stamp = now;
      pose.header.frame_id = "map";
      pose.pose.pose.orientation.x = q.x();
      pose.pose.pose.orientation.y = q.y();
      pose.pose.pose.orientation.z = q.z();
      pose.pose.pose.orientation.w = q.w();

      pub_pose_correction.publish(pose);
      ROS_INFO("Corrected pose to: %.1f", (desiredTheta + diff)*180/M_PI);
    }


    void run()
    {
        enum state state = FOLLOW;

        while (ros::ok())
        {
            if((ros::Time::now() - start_time).toSec() > time_until_path)
            {
                wallBrain = false;
                break;
            }

            ros::spinOnce();

            switch (state)
            {
            case ALIGN:
            {
                if (state_str == "LEFT")
                {
                    static bool align_done = false;
                    if (!align_done && align(1.4)) //align just finished
                    {
                        count = 0;
                        align_done = true;
                        poseCorrR = true;
                    }
                    else if (align_done && count == 1 &&
                            (distance.right_front >= maxWallAlignDistance ||
                             distance.right_rear >= maxWallAlignDistance))
                    {
                        ROS_INFO("skipping theta correction, distances: %.2f, %.2f > %.2f",
                                 distance.right_front, distance.right_rear, maxWallAlignDistance);
                        state = FOLLOW;
                        ros::Duration(0.4).sleep();
                        align_done = false;
                        poseCorrR = false;
                    }
                    else if (align_done && count >= 10)
                    {
                        if (std::abs(avgRightDiff) <= maxWallAlignAngle)
                            publishOdometry(ros::Time(), avgRightDiff);
                        else
                            ROS_INFO("skipping theta correction, angles: |%.2f| > %.2f",
                                     avgRightDiff, maxWallAlignAngle);
                        state = FOLLOW;
                        ros::Duration(0.4).sleep();
                        align_done = false;
                        poseCorrR = false;
                    }
                }
                else if (state_str == "RIGHT")
                {
                    static bool align_done = false;
                    if (!align_done && align(1.4)) //align just finished
                    {
                        count = 0;
                        align_done = true;
                        poseCorrL = true;
                    }
                    if (align_done && count == 1 &&
                            (distance.left_front >= maxWallAlignDistance ||
                             distance.left_rear >= maxWallAlignDistance))
                    {
                        ROS_INFO("skipping theta correction, distances: %.2f, %.2f > %.2f",
                                 distance.left_front, distance.left_rear, maxWallAlignDistance);
                        state = FOLLOW;
                        ros::Duration(0.4).sleep();
                        align_done = false;
                        poseCorrL = false;
                    }
                    if (align_done && count >= 10)
                    {
                        if (std::abs(avgLeftDiff) <= maxWallAlignAngle)
                            publishOdometry(ros::Time(), avgLeftDiff);
                        else
                            ROS_INFO("skipping theta correction, angles: |%.2f| > %.2f",
                                     avgLeftDiff, maxWallAlignAngle);
                        state = FOLLOW;
                        ros::Duration(0.4).sleep();
                        align_done = false;
                        poseCorrL = false;
                    }
                }

                break;
            }
            case FOLLOW:
                if (!follow(0.2, 0.15))
                {
                    if (distance.left_front > 0.2)
                        desiredTheta = theta+M_PI/2;
                    else
                        desiredTheta = theta-M_PI/2;
                    state = ALIGN;
                    ROS_INFO("state: ALIGN");
                    ros::Duration(0.2).sleep();

                }
                break;

            }

            loop_rate.sleep();
        }

        ROS_INFO("switching");

        ros::spin();

        ROS_INFO("exiting");
    }

    double angleDiff(double a, double b)
    {
        a = a-b;
        if (a > M_PI)
            a -= 2*M_PI;
        if (a < -M_PI)
            a += 2*M_PI;
        return a;
    }

    bool align(double speed)
    {
        double error = angles::shortest_angular_distance(theta, desiredTheta)*180/M_PI;

        if (std::abs(error) < 1)
        {
            geometry_msgs::Twist twist;
            pub_motor_twist.publish(twist);
            ros::Duration(0.5).sleep();
            return true;
        }
        else
        {
            geometry_msgs::Twist twist;
            twist.angular.z = clamp(error/4, -speed, speed);
            pub_motor_twist.publish(twist);
            return false;
        }
    }

    bool follow(double speed, double frontDistance)
    {
        geometry_msgs::Twist twist;
        if (distance.front > 0.35)
        {
            twist.linear.x = speed;
            twist.angular.z = 0;
            pub_wall_twist.publish(twist);
            return true;
        }
        else if (distance.front > frontDistance)
        {
            twist.linear.x = clamp(speed*0.9*(1+(distance.front-frontDistance)), 0.0, speed);
            pub_wall_twist.publish(twist);
            return true;
        }
        else
        {
            twist.linear.x = 0;
            twist.angular.z = 0;
            pub_wall_twist.publish(twist);
            ros::Duration(0.5).sleep();
            return false;
        }
    }
};
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pathfollower");
    wheatley::PathFollower path_following_node;
    path_following_node.run();
}

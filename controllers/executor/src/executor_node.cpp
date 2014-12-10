#include <ros/ros.h>
#include <sensors/Distance.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/tf.h>
#include <algorithm>
#include <sound_play/SoundRequest.h>
#include <executor/State.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <wheatley_common/common.h>
#include <angles/angles.h>


using std::string;

namespace wheatley
{
    class Executor : NiceBaseClass
    {
    private:
        ros::Publisher pub_motor_twist;
        ros::Publisher pub_wall_twist;
        ros::Publisher pub_state;
        ros::Publisher pub_pose_correction;

        ros::Subscriber sub_ir_distances;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_order;

        sound_play::SoundRequest speaker_msg;
        nav_msgs::Odometry pose;
        sensors::Distance distance;
        string state;


        tf::StampedTransform tf_left_front;
        tf::StampedTransform tf_left_rear;
        tf::StampedTransform tf_right_front;
        tf::StampedTransform tf_right_rear;


        tf::TransformListener tfl;
        ros::Rate loop_rate;
        double theta;
        double desiredTheta;
        double leftDiff [10];
        double rightDiff[10];
        double lR;
        double lL;
        double avgLeftDiff;
        double avgRightDiff;
        bool poseCorrL;
        bool poseCorrR;
        int count;
        bool crashL;
        bool crashR;
        bool crash;
        bool ready;
        double maxWallAlignDistance;
        double maxWallAlignAngle;



    public:
        Executor()
            : loop_rate(requireParameter<double>("rate"))
            , maxWallAlignDistance(requireParameter<double>("max_wall_align_distance"))
            , maxWallAlignAngle(requireParameter<double>("max_wall_align_angle")*M_PI/180)
            , poseCorrL(false)
            , poseCorrR(false)
            , desiredTheta(0)
            , state("STOP")
        {
            sub_ir_distances = nh.subscribe("/sensors/ir/distances", 1, &Executor::distanceCallback, this);
            sub_pose = nh.subscribe("/sensors/pose", 1, &Executor::poseCallback, this);
            sub_order = nh.subscribe("order", 1, &Executor::orderCallback, this);
            pub_state = nh.advertise<std_msgs::String>("state",1,true);
            pub_motor_twist = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
            pub_wall_twist = nh.advertise<geometry_msgs::Twist>("/wall_avoider/twist", 1);
            pub_pose_correction = nh.advertise<nav_msgs::Odometry> ("pose_correction", 1);

            ros::Time now(0);

            while (!tfl.waitForTransform("ir_front", "robot", now, ros::Duration(1)))
                ROS_ERROR("Couldn't find transform from 'robot' to 'ir_front', retrying...");

            tfl.lookupTransform("robot", "ir_left_front",  now, tf_left_front);
            tfl.lookupTransform("robot", "ir_left_rear",  now, tf_left_rear);
            tfl.lookupTransform("robot", "ir_right_front",  now, tf_right_front);
            tfl.lookupTransform("robot", "ir_right_rear",  now, tf_right_rear);
            lL = tf_left_front.getOrigin().distance(tf_left_rear.getOrigin());
            lR = tf_right_front.getOrigin().distance(tf_right_rear.getOrigin());
            ready = true;
            crashL = false;
            crashR = false;
            crash = false;
        }


        void orderCallback(const std_msgs::String::ConstPtr &msg)
        {
            if (state == "LEFT" || state == "RIGHT")
            {
                ROS_ERROR("ordered state transition %7s->%7s not allowed, ignoring...", state.data(), msg->data.data());
                return;
            }
            state = msg->data;

            if (state == "LEFT")
                desiredTheta = angles::normalize_angle(desiredTheta + M_PI/2);
            else if (state == "RIGHT")
                desiredTheta = angles::normalize_angle(desiredTheta - M_PI/2);
        }

        void distanceCallback(const sensors::Distance::ConstPtr &msg)
        {
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

            if (distance.bumper && state == "LEFT" && ready)
            {
                state = "BACK";
                crashL = true;
                ready = false;
            }
            else if (distance.bumper && state == "RIGHT" && ready)
            {
                state = "BACK";
                crashR = true;
                ready = false;
            } else if (distance.bumper && state == "FORWARD" && ready)
            {
                crash = true;
                ready = false;
                state = "BACK";
            }
        }

        void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
          pose = *msg;
          theta = tf::getYaw(msg->pose.pose.orientation);
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
            while (ros::ok())
            {
                static string prev_state = "";
                if (state == "FORWARD")
                {
                    if (!follow(0.2, 0.08))
                        state = "STOP";
                        ready = true;
                }
                else if (state == "STOP")
                {
                    if (prev_state != "STOP")
                    {
                        geometry_msgs::Twist twist;
                        twist.linear.x = 0;
                        twist.angular.z = 0;
                        pub_wall_twist.publish(twist);
                        ready = true;
                    }
                }
                else if (state == "BACK")
                {
                    static tf::Point prev_pos;
                    if (state != prev_state)
                        tf::pointMsgToTF(pose.pose.pose.position, prev_pos);

                    tf::Point curr_pos;
                    tf::pointMsgToTF(pose.pose.pose.position, curr_pos);

                    if (prev_pos.distance(curr_pos) < 0.02)
                    {
                        geometry_msgs::Twist twist;
                        twist.linear.x = -0.2;
                        pub_motor_twist.publish(twist);
                    }
                    else
                    {
                        geometry_msgs::Twist twist;
                        pub_motor_twist.publish(twist);
                        if (crashR)
                        {
                            state = "RIGHT";
                            crashR = false;
                        } else if (crashL)
                        {
                            state = "LEFT";
                            crashL = false;
                        }
                        else if (crash)
                        {
                            state = "LEFT";
                            crash = false;
                            desiredTheta = angles::normalize_angle(desiredTheta + M_PI/2);
                        } else
                        {
                            state = "STOP";
                        }
                    }
                }
                else if (state == "LEFT")
                {
                    static bool align_done = false;
                    if (!align_done && align(1.5)) //align just finished
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
                        state = "STOP";
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
                        state = "STOP";
                        align_done = false;
                        poseCorrR = false;
                        ready = true;
                    }
                }
                else if (state == "RIGHT")
                {
                    static bool align_done = false;
                    if (!align_done && align(1.5)) //align just finished
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
                        state = "STOP";
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
                        state = "STOP";
                        align_done = false;
                        poseCorrL = false;
                        ready = true;
                    }
                }

                if (prev_state != state)
                {
                    std_msgs::String msg_state;
                    msg_state.data = state;
                    pub_state.publish(msg_state);

                    ROS_INFO("%7s->%-7s: desiredTheta: %.1f, theta: %.1f",
                             prev_state.data(), state.data(),
                             desiredTheta*180/M_PI, theta*180/M_PI);

                    prev_state = state;
                }

                ros::spinOnce();
                loop_rate.sleep();
            }
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
                twist.angular.z = clamp(error/8, -speed, speed);
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
    ros::init(argc, argv, "executor");
    wheatley::Executor executor_node;
    executor_node.run();
}

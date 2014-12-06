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


using std::string;

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

namespace wheatley
{
    class Executor : NiceBaseClass
    {
    private:
      ros::Publisher motor_twist;
      ros::Publisher wall_twist;
      ros::Publisher pub_state;
      ros::Publisher pub_pose_correction;

      ros::Subscriber sub_distance;
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
      double alignment;
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
      std::string str_stat;



    public:
      Executor()
          : loop_rate(requireParameter<double>("rate"))
          , poseCorrL(false)
          , poseCorrR(false)
      {
        sub_distance = nh.subscribe("/sensors/ir/distances", 1, &Executor::distanceCallback, this);
        sub_pose = nh.subscribe("/sensors/pose", 1, &Executor::poseCallback, this);
        sub_order = nh.subscribe("order", 1, &Executor::orderCallback, this);
        pub_state = nh.advertise<std_msgs::String>("state",1,true);
        motor_twist = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        wall_twist = nh.advertise<geometry_msgs::Twist>("/wall_avoider/twist", 1);
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
      }


        void orderCallback(const std_msgs::String::ConstPtr &msg)
        {
            state = msg->data;
            if (state == "LEFT")
            {
                desiredTheta = fmod(desiredTheta + M_PI/2,(double) 2*M_PI);
                if (desiredTheta>M_PI/2) desiredTheta-=2*M_PI;
                    alignment = desiredTheta;
            }
            else if (state == "RIGHT")
            {
                desiredTheta = fmod (desiredTheta - M_PI/2, (double) 2*M_PI);
                if (desiredTheta>M_PI/2) desiredTheta-=2*M_PI;
                    alignment = desiredTheta;
            }
            else if (state == "FORWARD")
                ;
            else
                state = "STOP";
            ROS_INFO("Before state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
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
        } else if (poseCorrR)
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
        desiredTheta = 0;
        stat = STOP;
        geometry_msgs::Twist twist;

        int a=0;
        while (ros::ok())
        {
            if (state == "FORWARD")
            {
                  if (!follow(0.2, 0.10))
                  {
                    ROS_INFO("After state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
                    stat = STOP;
                  }
            }
            else if (state == "STOP")
            {
                twist.linear.x = 0;
                twist.angular.z = 0;
                motor_twist.publish(twist);
            }
            else if (state == "LEFT")
            {
                //TODO: check that the distance to the wall, on the last measurement from both sensors is reasonable
                static bool align_done = false;
                if (!align_done && align(0.2))
                {
                    count = 0;
                    poseCorrR = true;
                    align_done = true;
                }

                if (align_done && count >= 10)
                {
                    if (std::abs(avgRightDiff) < (10*M_PI)/180 && distance.right_front < 0.15 && distance.right_rear < 0.15)
                        publishOdometry(ros::Time(), avgRightDiff);
                    ROS_INFO("AvgRightDiff: %f",avgRightDiff);
                    ROS_INFO("After state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
                    stat = STOP;
                    poseCorrR = false;
                    align_done = false;
                }
            }
            else if (state == "RIGHT")
            {
                //TODO: check that the distance to the wall, on the last measurement from both sensors is reasonable
                static bool align_done = false;
                if (!align_done && align(0.2))
                {
                    count = 0;
                    poseCorrL = true;
                    align_done = true;
                }

                if (align_done && count >= 10)
                {
                    if (std::abs(avgLeftDiff) < (10*M_PI)/180 && distance.left_front < 0.15 && distance.left_rear < 0.15)
                        publishOdometry(ros::Time(), avgLeftDiff);
                    ROS_INFO("AvgRightDiff: %f",avgLeftDiff);
                    ROS_INFO("After state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
                    stat = STOP;
                    poseCorrL = false;
                    align_done = false;
                }
            }
          }

          static string prev_state = "";
          if (prev_state != state)
          {
              std_msgs::String msg_state;
              prev_state = state;
              msg_state.data = state;
              pub_state.publish(msg_state);
          }

          ros::spinOnce();
          loop_rate.sleep();
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
        double error = angleDiff(alignment, theta);

        if (std::abs(error) < 1*M_PI/180)
        {
            geometry_msgs::Twist twist;
            motor_twist.publish(twist);
            ros::Duration(0.5).sleep();
            return true;
        }
        else
        {
            geometry_msgs::Twist twist;
            twist.angular.z = CLAMP(10*error, -speed, speed);
            motor_twist.publish(twist);
            return false;
        }
     }

      bool follow(double speed, double frontDistance)
      {
        geometry_msgs::Twist twist;
        if (distance.front > 0.30)
        {
            twist.linear.x = speed;
            wall_twist.publish(twist);
            return true;
        }
        else if (distance.front > frontDistance)
        {
            twist.linear.x = speed/2;
            wall_twist.publish(twist);
            return true;
        }
        else
        {
            twist.linear.x = 0;
            geometry_msgs::Twist twist;
            wall_twist.publish(twist);
            ros::Duration(0.5).sleep();
            return false;
        }
      }
    };
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "executor");
    Executor executor_node;
    executor_node.run();
}

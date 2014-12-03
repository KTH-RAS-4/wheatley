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




template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

enum state { FORWARD, LEFT, RIGHT, STOP };
enum state stat;

class Executor
{
private:
  ros::NodeHandle n;
  ros::NodeHandle n1;
  ros::Publisher motor_twist;
  ros::Publisher wall_twist;
  ros::Publisher pub_task_done;
  ros::Publisher pub_pose_correction;

  ros::Subscriber sub_distance;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_action;

  sound_play::SoundRequest speaker_msg;
  nav_msgs::Odometry pose;
  sensors::Distance distance;
  std_msgs::String STATE;


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
    : n("~")
    , loop_rate(10)
  {
    init();
  }

  ~Executor()
  {
  }

  void init()
  {
    double rate = 100;
    //n.getParam("rate", rate);
    loop_rate = ros::Rate(rate);

    sub_distance = n.subscribe("/sensors/ir/distances", 1000, &Executor::distanceCallback, this);
    sub_pose = n.subscribe("/sensors/pose", 1000, &Executor::poseCallback, this);
    sub_action = n.subscribe("order", 1000, &Executor::orderCallback, this);
    pub_task_done = n.advertise<std_msgs::String>("state",1000,true);
    motor_twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
    wall_twist = n.advertise<geometry_msgs::Twist>("/wall_avoider/twist", 1000);
    pub_pose_correction = n.advertise<nav_msgs::Odometry> ("pose_correction", 1);

    ros::Time now(0);

    while (!tfl.waitForTransform("ir_front", "robot", now, ros::Duration(1)))
        ROS_ERROR("Couldn't find transform from 'robot' to 'ir_front', retrying...");

    tfl.lookupTransform("robot", "ir_left_front",  now, tf_left_front);
    tfl.lookupTransform("robot", "ir_left_rear",  now, tf_left_rear);
    tfl.lookupTransform("robot", "ir_right_front",  now, tf_right_front);
    tfl.lookupTransform("robot", "ir_right_rear",  now, tf_right_rear);
    lL = tf_left_front.getOrigin().distance(tf_left_rear.getOrigin());
    lR = tf_right_front.getOrigin().distance(tf_right_rear.getOrigin());

    poseCorrL = poseCorrR = false;


  }


  void orderCallback(const std_msgs::String::ConstPtr &msg)
    {
    STATE = *msg;
    if (STATE.data == "LEFT")
      {
        stat = LEFT;
        ROS_INFO("LEFT");
        desiredTheta = fmod(desiredTheta + M_PI/2,(double) 2*M_PI);
        if (desiredTheta>M_PI/2) desiredTheta-=2*M_PI;
        alignment = desiredTheta;
        ROS_INFO("Before state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
      } else if (STATE.data == "RIGHT")
      {
        stat = RIGHT;
        ROS_INFO("RIGHT");
        desiredTheta = fmod (desiredTheta - M_PI/2, (double) 2*M_PI);
        if (desiredTheta>M_PI/2) desiredTheta-=2*M_PI;
        alignment = desiredTheta;
        ROS_INFO("Before state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
      } else if (STATE.data == "FORWARD")
      {
          stat = FORWARD;
          ROS_INFO("FORWARD");
          ROS_INFO("Before state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
      } else
      {
          stat = STOP;
          ROS_INFO("STOP");
      }
    }

  void distanceCallback(const sensors::Distance::ConstPtr &msg)
  {
    distance = *msg;
    if (poseCorrL)
    {
        if (count >= 10)
        {
            double sumL;
            for(int i = 1; i < 10; i++)
            {
               sumL += leftDiff[i];
            }
            avgLeftDiff = sumL / (double) 10;
        } else {
            leftDiff[count++] = atan(-(distance.left_front-distance.left_rear)/lL);
        }
    } else if (poseCorrR)
    {
        if (count >= 10)
        {
            double sumR;
            for(int i = 1; i < count; i++)
            {
               sumR += rightDiff[i];
            }
            avgRightDiff = sumR / (double) 10;
        } else {
            rightDiff[count++] = atan((distance.right_front-distance.right_rear)/lR);
        }
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
      switch (stat)
      {
          case FORWARD:
              if (!follow(0.16, 0.10))
              {
                ROS_INFO("After state, DesiredTheta: %.1f, Theta: %.1f",desiredTheta*180/M_PI, theta*180/M_PI);
                stat = STOP;
              }
              break;
          case STOP:
            twist.linear.x = 0;
            twist.angular.z = 0;
            wall_twist.publish(twist);
            break;

          case LEFT:
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
            break;
          }
          case RIGHT:
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
            break;
          }

      }
      if (stat == STOP)
      {
          str_stat = "STOP";
      } else if (stat == FORWARD)
      {
          str_stat = "FORWARD";
      } else if (stat == LEFT)
      {
          str_stat = "LEFT";
      } else if (stat == RIGHT)
      {
          str_stat = "RIGHT";
      }

      //static state prev_state = FORWARD;
      static std::string prev_state;
      if (prev_state != str_stat)
      {
          std_msgs::String curr_STATE;
          prev_state = str_stat;
          curr_STATE.data = str_stat;
          pub_task_done.publish(curr_STATE);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
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
        //geometry_msgs::Twist twist;
        twist.linear.x = speed;
        wall_twist.publish(twist);
        return true;
    }
    else if (distance.front > frontDistance)
    {
        //geometry_msgs::Twist twist;
        twist.linear.x = speed/2;
        wall_twist.publish(twist);
        return true;
    }
    else
    {
        //geometry_msgs::Twist twist;
        twist.linear.x = 0;
        geometry_msgs::Twist twist;
        wall_twist.publish(twist);
        ros::Duration(0.5).sleep();
        return false;
    }
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "executor");
  Executor my_node;
  my_node.run();
}

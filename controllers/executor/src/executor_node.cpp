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

  ros::Subscriber sub_distance;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_action;

  sound_play::SoundRequest speaker_msg;
  nav_msgs::Odometry pose;
  sensors::Distance distance;
  executor::State STATE;
  ros::Rate loop_rate;
  double alignment;
  double theta;
  double desiredTheta;

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
    double rate = 10;
    //n.getParam("rate", rate);
    loop_rate = ros::Rate(rate);

    sub_distance = n.subscribe("/sensors/ir/distances", 1000, &Executor::distanceCallback, this);
    sub_pose = n.subscribe("/sensors/pose", 1000, &Executor::poseCallback, this);
    sub_action = n.subscribe("/nav/order", 1000, &Executor::orderCallback, this);
    pub_task_done = n.advertise<executor::State>("executor/done",1000);
    motor_twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
    wall_twist = n.advertise<geometry_msgs::Twist>("/wall_avoider/twist", 1000);



  }

  void orderCallback(const executor::State::ConstPtr &msg)
    {
    STATE = *msg;
    if (STATE.state == "LEFT")
      {
        stat = LEFT;
        alignment = theta+M_PI/2;
      } else if (STATE.state == "RIGHT")
      {
        stat = RIGHT;
        alignment = theta-M_PI/2;
      } else if (STATE.state == "FORWARD")
    {
        stat = FORWARD;
    } else
    {
        stat = STOP;
    }
    }

  void distanceCallback(const sensors::Distance::ConstPtr &msg)
  {
    distance = *msg;
  }

  void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
      pose = *msg;
      theta = tf::getYaw(msg->pose.pose.orientation);
  }

  void run()
  {
    stat = STOP;
    geometry_msgs::Twist twist;

    int a=0;
    while (ros::ok())
    {

      ros::spinOnce();
      switch (stat)
      {
      case FORWARD:
          ROS_INFO("FORWARD");
          if (!follow(0.16, 0.15))
          {
            ros::Duration(0.5).sleep();
            pub_task_done.publish(STATE);
            stat = STOP;
          }
          break;
      case STOP:
        ROS_INFO("STOP");
        twist.linear.x = 0;
        twist.angular.z = 0;
        wall_twist.publish(twist);
        break;

      case LEFT:
        ROS_INFO("LEFT");
          if (align(0.2))
        {
            ros::Duration(0.5).sleep();
            pub_task_done.publish(STATE);
            stat = STOP;
        }
        break;
      case RIGHT:
      ROS_INFO("RIGHT");
	  if (align(0.2))
          {
              ros::Duration(0.5).sleep();
              pub_task_done.publish(STATE);
              stat = STOP;
          }
          break;

      }


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

    if (std::abs(error) < 3*M_PI/180)
    {
        geometry_msgs::Twist twist;
        motor_twist.publish(twist);
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
        return false;
    }
  }
};

int main (int argc, char **argv){
  ros::init(argc, argv, "wall_brain");
  Executor my_node;
  my_node.run();
}

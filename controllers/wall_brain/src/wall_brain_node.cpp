#include <ros/ros.h>
#include <sensors/Distance.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/tf.h>
#include <algorithm>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

enum state { FOLLOW, ALIGN, FIND_ALIGNMENT };

class WallBrain
{
private:
  ros::NodeHandle n;
  ros::Publisher motor_twist;
  ros::Publisher wall_twist;
  ros::Subscriber sub_distance;
  ros::Subscriber sub_pose;

  nav_msgs::Odometry pose;
  sensors::Distance distance;
  ros::Rate loop_rate;
  double alignment;
  double theta;

public:
  WallBrain()
    : n("~")
    , loop_rate(10)
  {
    init();
  }

  ~WallBrain()
  {
  }

  void init()
  {
    double rate = 10;
    //n.getParam("rate", rate);
    loop_rate = ros::Rate(rate);

    sub_distance = n.subscribe("/sensors/distance", 1000, &WallBrain::distanceCallback, this);
    sub_pose = n.subscribe("/sensors/pose", 1000, &WallBrain::poseCallback, this);
    motor_twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
    wall_twist = n.advertise<geometry_msgs::Twist>("/wall_avoider/twist", 1000);
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
    enum state state = FIND_ALIGNMENT;
    ROS_INFO("state: FIND_ALIGNMENT");


    while (ros::ok())
    {
      ros::spinOnce();

      switch (state)
      {
      case FIND_ALIGNMENT:
          if (findAlignment(0.2))
          {
              state = ALIGN;
              ros::Duration(0.5).sleep();
          }
          break;
      case ALIGN:
          if (align(0.3))
          {
              state = FOLLOW;
              ros::Duration(0.2).sleep();
              ROS_INFO("state: FOLLOW");
          }
          break;
      case FOLLOW:
          if (!follow(0.3, 15))
          {
              if (distance.right_front > 15)
                  alignment = theta-M_PI/2;
              else
                  alignment = theta+M_PI/2;

              state = ALIGN;
              ros::Duration(0.2).sleep();
          }
          break;
      }

      loop_rate.sleep();
    }
  }

  bool findAlignment(double speed)
  {
    static double l = 0;
    static double r = 0;

    double l2 = distance.left_front-distance.left_rear;
    double r2 = distance.right_front-distance.right_rear;
    if (distance.left_front > 15)
        l2 = 0;
    if (distance.right_front > 15)
        r2 = 0;
    //ROS_INFO("l %.1f r %.1f   l2 %.2f r2 %.2f", l, r, l2, r2);

    if ((sgn(l) != sgn(l2) && l != 0 && l2 != 0) ||
        (sgn(r) != sgn(r2) && r != 0 && r2 != 0))
    {
        alignment = theta;
        geometry_msgs::Twist twist;
        motor_twist.publish(twist);
        return true;
    }
    else
    {
        l = l2;
        r = r2;
        geometry_msgs::Twist twist;
        twist.angular.z = speed;
        motor_twist.publish(twist);
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
    ROS_INFO("state: ALIGN ");

    if (std::abs(error) < 3*M_PI/180)//error should be in radians. 3 degres=3*3.1415/180 rad = 0.05 rad
    {
        ROS_INFO("state: ****************TURNED 90 DEGREES COMPLETE ******************");
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
    if (distance.front > 30)
    {
        //geometry_msgs::Twist twist;
        twist.linear.x = speed;
        wall_twist.publish(twist);
        return true;
    }
    else if (distance.front > frontDistance)
    {
        //geometry_msgs::Twist twist;
        twist.linear.x = 0.10;
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
int main (int argc, char **argv)
{
  ros::init(argc, argv, "wall_brain");
  WallBrain my_node;
  my_node.run();
}

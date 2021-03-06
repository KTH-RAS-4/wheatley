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
#include <angles/angles.h>
#include <wheatley_common/common.h>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

enum state { FOLLOW, ALIGN, FIND_ALIGNMENT };
namespace wheatley {

class WallBrain : NiceBaseClass
{
private:
  ros::NodeHandle n;
  ros::NodeHandle n1;
  ros::Publisher pub_motor_twist;
  ros::Publisher pub_wall_twist;

  ros::Subscriber sub_distance;
  ros::Subscriber sub_pose;

  sound_play::SoundRequest speaker_msg;
  nav_msgs::Odometry pose;
  sensors::Distance distance;
  ros::Rate loop_rate;
  double desiredTheta;
  double theta;
  double minTurnTwist;

public:
  WallBrain()
    : n("~")
    , loop_rate(10)
    , minTurnTwist(requireParameter<double>("minTurnTwist"))
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

    sub_distance = n.subscribe("/sensors/ir/distances", 1000, &WallBrain::distanceCallback, this);
    sub_pose = n.subscribe("/sensors/pose", 1000, &WallBrain::poseCallback, this);
    pub_motor_twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
    pub_wall_twist = n.advertise<geometry_msgs::Twist>("/wall_avoider/twist", 1000);



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
    enum state state = FOLLOW;
    ROS_INFO("state: FIND_ALIGNMENT");

    int a=0;
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
          if (align(1.4))
            {
              state = FOLLOW;
              ROS_INFO("state: FOLLOW");
              ros::Duration(0.2).sleep();
          }
          break;
      case FOLLOW:
          if (!follow(0.2, 0.15))
          {
              if (distance.right_front > 0.2)
                  desiredTheta = theta-M_PI/2;
              else
                  desiredTheta = theta+M_PI/2;
              state = ALIGN;
              ROS_INFO("state: ALIGN");
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
    if (distance.left_front > 0.15)
        l2 = 0;
    if (distance.right_front > 0.15)
        r2 = 0;
    //ROS_INFO("l %.1f r %.1f   l2 %.2f r2 %.2f", l, r, l2, r2);

    if ((sgn(l) != sgn(l2) && l != 0 && l2 != 0) ||
        (sgn(r) != sgn(r2) && r != 0 && r2 != 0))
    {
        desiredTheta = theta;
        geometry_msgs::Twist twist;
        pub_motor_twist.publish(twist);
        return true;
    }
    else
    {
        l = l2;
        r = r2;
        geometry_msgs::Twist twist;
        twist.angular.z = speed;
        pub_motor_twist.publish(twist);
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
          twist.angular.z = clamp(error/3.5, -speed, speed);
          if (0 <= twist.angular.z && twist.angular.z < minTurnTwist)
          {
              twist.angular.z = minTurnTwist;
          } else if (-minTurnTwist < twist.angular.z && twist.angular.z < 0)
          {
              twist.angular.z = -minTurnTwist;
          }
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
int main (int argc, char **argv){
  ros::init(argc, argv, "wall_brain");
  wheatley::WallBrain my_node;
  my_node.run();
}


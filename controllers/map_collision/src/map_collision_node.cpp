#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/tf.h>
#include <algorithm>
#include <sound_play/SoundRequest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <map>


template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

namespace gu=occupancy_grid_utils;
using std::string;
namespace gm=geometry_msgs;

class map_collision
{
private:

  ros::NodeHandle n1;
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Publisher pub_speaker;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_map;
  sound_play::SoundRequest speaker_msg;
  nav_msgs::Odometry pose;
  ros::Rate loop_rate;
  double theta,robot_x,robot_y;

  //gu::OverlayClouds map;
  nav_msgs::OccupancyGrid map;


public:
  map_collision()
    : n("~")
    , loop_rate(10)
  {
    init();
  }

  ~map_collision()
  {
  }
  gm::Pose robot_pose;
  double robot_outer_diameter;
  void init()
  {
    double rate = 10;
    loop_rate = ros::Rate(rate);
    sub_pose = n.subscribe("/sensors/pose", 1000, &map_collision::poseCallback, this);
    sub_map = nh.subscribe("map_in", 1, &map_collision::callback_map, this);
    robot_outer_diameter = 0.255;

  }

  void callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
      map = *msg;
      /*
      inflated_map = *gu::inflateObstacles(map, robot_diameter/2);
      inflated_map.header.frame_id = msg->header.frame_id;
      inflated_map.header.stamp = msg->header.stamp;
      pub_map.publish(inflated_map);
      */
      //map = occupancy_grid_utils::createCloudOverlay(msg, fixed_frame, 0.1, 10, 1);
  }

  void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
      theta = tf::getYaw(msg->pose.pose.orientation);
      robot_pose.position.x = msg->pose.pose.position.x;
      robot_pose.position.y = msg->pose.pose.position.y;
  }

  void run()
  {
      int n=1,z=5;
      bool answer;
    while (ros::ok())
    {
      ros::spinOnce();
      //ROS_INFO("run");

      n++;
      if (n==10)
     {
       //if (IsWindowFree(z))
      ROS_INFO("x=[%f] y=[%f] theta=[%f]",robot_pose.position.x,robot_pose.position.y,theta);

      //gu::addKnownFreePoint(&map, robot_pose.position, robot_outer_diameter/2);

      //void addKnownFreePoint (OverlayClouds* overlay, const gm::Point& p, const double r)
      //void addKnownOccupiedPoint (OverlayClouds* overlay, const gm::Point& p, const double r)
      //bool IsWindowFree (OverlayClouds* overlay, const gm::Point& p, const double r)

      n=0;
    }

      loop_rate.sleep();


    }
  }

  bool IsWindowFree (int a)
  {
     for (int i=a-3; i<=a;i++)
     {
        if (!((i%10)<=3))
        {
              return false;
          }
    }
    ROS_INFO("a=%d ",a);
    return true;
  }

};

int main (int argc, char **argv){
  ros::init(argc, argv, "map_collision");
  map_collision my_node;
  my_node.run();
}

#include <ros/ros.h>
#include <ras_arduino_msgs/Distance.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
  return value < low ? low : (value > high ? high : value);
}

float lerp(float a, float b, float m)
{
    return (a*m+b*(1.0f-m));
}

class WallFollowingControllerNode
{
private:
  ras_arduino_msgs::Distance dist;
  geometry_msgs::Twist twist_in;

  ros::NodeHandle n;
  ros::Publisher pub_twist;
  ros::Subscriber sub_twist;
  ros::Subscriber sub_distance;

  ras_arduino_msgs::Distance distance;
  ros::Rate loop_rate;

public:
  WallFollowingControllerNode()
    : n("~")
    , loop_rate(1)
  {
    init();
  }
  
  ~WallFollowingControllerNode()
  {
  }
  
  void init()
  {
    double rate;
    n.getParam("rate", rate);
    loop_rate = ros::Rate(rate);

    sub_distance = n.subscribe("/sensors/distance", 1000, &WallFollowingControllerNode::distanceCallback, this);
    sub_twist = n.subscribe("twist_in", 1000, &WallFollowingControllerNode::twistCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("twist_out", 1000);
  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    twist_in.linear.x = msg->linear.x;
    twist_in.angular.z = msg->angular.z;
  }

  void distanceCallback(const ras_arduino_msgs::Distance::ConstPtr &msg)
  {
    distance = *msg;
  }

  void run()
  {
    while (ros::ok())
    {
      ros::spinOnce();
      calc();
      loop_rate.sleep();
    }
  }



  void calc()
  {
    geometry_msgs::Twist twist_out;

    float x = CLAMP(5-distance.right_front, 0, 1);

    twist_out.linear.x = twist_in.linear.x; //stop when close in the front
    twist_out.angular.z = (1-x)*twist_in.angular.z + 2*x;

    ROS_INFO("twist_in:  [%6.1f, %6.1f]", twist_in.linear.x, twist_in.angular.z);
    ROS_INFO("twist_out: [%6.1f, %6.1f]", twist_out.linear.x, twist_out.angular.z);

    pub_twist.publish(twist_out);
  }
};
int main (int argc, char **argv)
{
  ros::init(argc, argv, "wall_following_controller_node");
  WallFollowingControllerNode my_node;
  my_node.run();
}

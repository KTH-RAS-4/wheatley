#include <ros/ros.h>
#include <sensors/Distance.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>

float IR = 0.0;
float IL = 0.0;
float last_errorL;
float last_errorR;
bool left = false;
bool right = false;
double PIDR;
double PIDL;


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
  geometry_msgs::Twist twist_in;

  ros::NodeHandle n;
  ros::Publisher pub_twist;
  ros::Subscriber sub_twist;
  ros::Subscriber sub_distance;


  sensors::Distance distance;
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

    double rate = 100;
    //n.getParam("rate", rate);
    loop_rate = ros::Rate(rate);
    sub_distance = n.subscribe("/sensors/ir/distances", 1, &WallFollowingControllerNode::distanceCallback, this);
    sub_twist = n.subscribe("twist_in", 1, &WallFollowingControllerNode::twistCallback, this);
    pub_twist = n.advertise<geometry_msgs::Twist>("twist_out", 1);
  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    twist_in = *msg;
    calc();
  }

  void distanceCallback(const sensors::Distance::ConstPtr &msg)
  {
    distance = *msg;
  }

  void run()
  {
    ros::spin();
    //calc();
  }

  void calc()
  {
    geometry_msgs::Twist twist_out;
    float minDistance = 0.03;
    float minFrontDistance = 0.08;
    float minStraight = 0.005;
    float small_turn = 0.35;//.15
    //ROS_INFO("small_turn:  [%f]", small_turn);
    float errorL = minDistance - distance.left_front;
    float errorR = minDistance - distance.right_front;
    float Kp = 10;
    float Ki = 1;
    float Kd = 0.4;
    double elapsed = loop_rate.expectedCycleTime().toSec();
    float DL = (errorL - last_errorL) / elapsed;
    float DR = (errorR - last_errorR) / elapsed;
    IL += errorL*elapsed;
    IR += errorR*elapsed;
    last_errorL = errorL;
    last_errorR = errorR;

    if (elapsed > 0.12 || twist_in.linear.x == 0)
    {
        PIDL = 0;
        PIDR = 0;
        IL = 0;
        IR = 0;
    }

    if (distance.right_front < minDistance)
    {
        PIDR = Kp*errorR + Ki*IR + Kd*DR;
        ROS_INFO_STREAM("right init");
        right = true;
    }
    else if (right && distance.right_front <= minDistance + 0.03)
    {
        PIDR = Kp*errorR + Ki*IR + Kd*DR;
        ROS_INFO_STREAM("right continue");
    }
    else if (right && distance.right_front > minDistance + 0.03)
    {
        twist_out = twist_in;
        right= false;
        IR = 0;
        PIDR = 0;
        ROS_INFO_STREAM("right end");
    }
    if (distance.left_front < minDistance)
    {
        PIDL = -(Kp*errorL + Ki*IL + Kd*DL);
        left = true;
        ROS_INFO_STREAM("left init");
    }
    else if (left && distance.left_front <= minDistance + 0.03)
    {
        PIDL = -(Kp*errorL + Ki*IL + Kd*DL);
        ROS_INFO_STREAM("left continue");
    }
    else if (left && distance.left_front > minDistance + 0.03)
    {
        twist_out = twist_in;
        left = false;
        IL = 0;
        PIDL = 0;
        ROS_INFO_STREAM("left end");
    }

    if ((left || right) && twist_in.linear.x != 0)
    {
        twist_out.angular.z = PIDL + PIDR;
        twist_out.linear.x = twist_in.linear.x;
        ROS_INFO_STREAM("fix");
    } else
    {
        twist_out = twist_in;
    }

    //ROS_INFO("twist_in:  [%6.1f, %6.1f]", twist_in.linear.x, twist_in.angular.z);
    //ROS_INFO("twist_out: [%6.1f, %6.1f]", twist_out.linear.x, twist_out.angular.z);

    pub_twist.publish(twist_out);
  }
};
int main (int argc, char **argv)
{
  ros::init(argc, argv, "wall_following_controller_node");
  WallFollowingControllerNode my_node;
  my_node.run();
}

#include <ros/ros.h>
#include <ras_arduino_msgs/Distance.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>

int left = 0;
int right = 0;

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
    twist_in = *msg;
    //twist_in.linear.x = msg->linear.x;
    //twist_in.angular.z = msg->angular.z;
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

    float minDistance = 5;
    float minStraight = 0.5;

    if (distance.right_front < minDistance && distance.right_front < distance.right_rear)
    {
        twist_out.linear.x = twist_in.linear.x;
        twist_out.angular.z = 0.15;
        right = 1;
    }
    else if (right == 1 && distance.right_front >  minDistance && distance.right_rear > minDistance)
    {
        if(distance.right_front > distance.right_rear && abs(distance.right_front-distance.right_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = -0.15;
        }
        else if(distance.right_front < distance.right_rear && abs(distance.right_front-distance.right_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = 0.15;
        }
        else
            twist_out = twist_in;
            right = 0;
    }
    else if (right == 1 &&  std::abs(distance.right_front-distance.right_rear) < minStraight)
    {
        twist_out = twist_in;
        right= 0;
    }
    else if (distance.left_front < minDistance && distance.left_front < distance.left_rear)
    {
        twist_out.linear.x = twist_in.linear.x;
        twist_out.angular.z = -0.15;
        left = 1;
    }
    else if (left == 1 && distance.left_front >  minDistance && distance.left_rear > minDistance)
    {
        if(distance.left_front > distance.left_rear && abs(distance.left_front-distance.left_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = 0.15;
        }
        else if(distance.left_front < distance.left_rear && abs(distance.left_front-distance.left_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = -0.15;
        }
        else
            twist_out = twist_in;
            left = 0;
    }
    else if (left == 1 &&  std::abs(distance.left_front-distance.left_rear) < minStraight)
    {
        twist_out = twist_in;
        left = 0;
    }
    else
        twist_out = twist_in;


    //float x = CLAMP(5-distance.right_front, 0, 1);

    //twist_out.linear.x = twist_in.linear.x; //stop when close in the front
    //twist_out.angular.z = (1-x)*twist_in.angular.z + 2*x;

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

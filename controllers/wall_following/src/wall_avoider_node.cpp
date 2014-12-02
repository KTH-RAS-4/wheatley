#include <ros/ros.h>
#include <sensors/Distance.h>
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
  geometry_msgs::Twist twist_in;

  ros::NodeHandle n;
  ros::Publisher pub_twist;
  ros::Subscriber sub_twist;
  ros::Subscriber sub_distance;


  sensors::Distance distance;

public:
  WallFollowingControllerNode()
    : n("~")
  {
    init();
  }
  
  ~WallFollowingControllerNode()
  {
  }
  
  void init()
  {
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

    float minDistance = 0.05;
    float minFrontDistance = 0.08;
    float minStraight = 0.005;
    float small_turn = 0.35;//.15
    ROS_INFO("small_turn:  [%f]", small_turn);

    //move forward and turn sligthly to right
    if (distance.right_front < minDistance && distance.right_front < distance.right_rear)
    {
        twist_out.linear.x = twist_in.linear.x;
        twist_out.angular.z = small_turn;
        right = 1;
    }
    //check if it has turn to much to the rigth, and alinght the sensors at margin: minStraight distance
    else if (right == 1 && distance.right_front >  minDistance && distance.right_rear > minDistance)
    {
        //for over turning, correct by tuning to left. no linear speed until is straigth
        if(distance.right_front > distance.right_rear && abs(distance.right_front-distance.right_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = -small_turn;
        }
         //only turn to right
        else if(distance.right_front < distance.right_rear && abs(distance.right_front-distance.right_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = small_turn;
        }
        //move as twist_in
        else
            twist_out = twist_in;
            right = 0;
    }
    //
    else if (right == 1 &&  std::abs(distance.right_front-distance.right_rear) < minStraight)
    {
        twist_out = twist_in;
        right= 0;
    }
    else if (distance.left_front < minDistance && distance.left_front < distance.left_rear)
    {
        twist_out.linear.x = twist_in.linear.x;
        twist_out.angular.z = -small_turn;
        left = 1;
    }
    else if (left == 1 && distance.left_front >  minDistance && distance.left_rear > minDistance)
    {
        if(distance.left_front > distance.left_rear && abs(distance.left_front-distance.left_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = small_turn;
        }
        else if(distance.left_front < distance.left_rear && abs(distance.left_front-distance.left_rear) > minStraight)
        {
            twist_out.linear.x = 0;
            twist_out.angular.z = -small_turn;
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

    if (distance.front < minFrontDistance)
    {
        twist_out.linear.x = 0;
        twist_out.angular.z = 0;
    }

    //float x = CLAMP(0.05-distance.right_front, 0, 0.01);

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

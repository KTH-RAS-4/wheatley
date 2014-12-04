#include "ros/ros.h"
#include "std_msgs/String.h"
#include "list"
#include "string"

using namespace std;

ros::Subscriber sub;
ros::Publisher pub;

list<string> queue;

void callback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "STOP" && !queue.empty()) {
        ROS_INFO_STREAM("Publishing " <<  queue.front());
        std_msgs::String state;
        state.data = queue.front();
        pub.publish(state);
        queue.pop_front();
    }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "navigation_queue_node");
  ros::NodeHandle n;


  sub = n.subscribe("/executor/state", 1, callback);
  pub = n.advertise<std_msgs::String>("/executor/order", 1);

  queue.push_back("LEFT");
  queue.push_back("LEFT");
  queue.push_back("LEFT");
  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("RIGHT");
  queue.push_back("FORWARD");
  queue.push_back("RIGHT");
  queue.push_back("FORWARD");
  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("RIGHT");
  queue.push_back("FORWARD");
  queue.push_back("RIGHT");
  queue.push_back("FORWARD");
  queue.push_back("RIGHT");
  queue.push_back("RIGHT");
  queue.push_back("FORWARD");

  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("RIGHT");
  queue.push_back("FORWARD");
  queue.push_back("RIGHT");
  queue.push_back("LEFT");
  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("LEFT");
  queue.push_back("FORWARD");
  queue.push_back("LEFT");

  ros::spin();

  return 0;
}

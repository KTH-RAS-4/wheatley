#include <ros/ros.h>


class Navigater
{
private:
  ros::NodeHandle nh;

public:
  Navigater()
    : nh("~")
  {
  }

  void run()
  {
      ros::spin();
 }
};


int main (int argc, char **argv){
  ros::init(argc, argv, "navigater");
  Navigater navigation_node;
  navigation_node.run();
}

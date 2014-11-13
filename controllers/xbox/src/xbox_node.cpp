#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>


class Xbox
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_twist;
    ros::Subscriber sub_joy;
    sensor_msgs::Joy joy;
    ros::Rate loop_rate;
    double linearConstant;
    double angularConstant;

public:
    Xbox()
        : nh("~")
        , loop_rate(1)
    {
        double rate;
        nh.getParam("rate", rate);
        loop_rate = ros::Rate(rate);

        nh.getParam("linear_constant", linearConstant);
        nh.getParam("angular_constant", angularConstant);

        pub_twist = nh.advertise<geometry_msgs::Twist>("twist", 1);
        sub_joy = nh.subscribe("joy", 1, &Xbox::joyCallback, this);
    }

    ~Xbox()
    {
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy = *msg;
        calc();
    }

    void calc()
    {
        geometry_msgs::Twist twist;

        twist.linear.x = linearConstant * joy.axes[1];
        twist.angular.z = angularConstant * joy.axes[3];

        pub_twist.publish(twist);
    }

    void run()
    {
        ros::spin();
        /*while(ros::ok())
        {
          ros::spinOnce();
          calc();
          loop_rate.sleep();
        }*/
    }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "xbox_node");
  Xbox xbox_node;
  xbox_node.run();
}

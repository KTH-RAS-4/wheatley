#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>


class Xbox
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_twist;
    ros::Subscriber sub_joy;
    geometry_msgs::Twist twist;

public:
    Xbox()
        : nh("~")
    {
        init();
    }

    ~Xbox()
    {
    }

    void init()
    {
        pub_twist = nh.advertise<geometry_msgs::Twist>("twist", 1000);
        sub_joy = nh.subscribe("joy", 1, &Xbox::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        twist.linear.x = joy->axes[1];
        twist.angular.z = 2*joy->axes[0];
    }

    void calc()
    {
        if (twist.linear.x < 0.1 && twist.linear.x > -0.1)
        {
            twist.linear.x = 0;
        }
        if (twist.angular.z < 0.1 && twist.angular.z > -0.1)
        {
            twist.angular.z = 0;
        }

        pub_twist.publish(twist);
    }

    void run()
    {
        ros::Rate loop_rate(10.0);
        while(ros::ok())
        {
          ros::spinOnce();
          calc();
          loop_rate.sleep();
        }
    }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "xbox_node");
  Xbox xbox_node;
  xbox_node.run();
}

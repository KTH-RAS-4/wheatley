#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <cmath>


class Xbox
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_twist;
    ros::Publisher pub_direction;
    ros::Subscriber sub_joy;
    ros::Timer timer_min_rate;
    sensor_msgs::Joy joy;
    double linearConstant;
    double angularConstant;

public:
    Xbox()
        : nh("~")
    {
        double min_rate;

        nh.getParam("linear_constant", linearConstant);
        nh.getParam("angular_constant", angularConstant);
        nh.getParam("min_rate", min_rate);

        pub_twist = nh.advertise<geometry_msgs::Twist>("twist", 1);
        pub_direction = nh.advertise<std_msgs::String>("direction", 1);

        sub_joy = nh.subscribe("joy", 1, &Xbox::joyCallback, this);
        timer_min_rate = nh.createTimer(ros::Rate(min_rate), &Xbox::timer, this);
    }

    ~Xbox()
    {
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        joy = *msg;
        calc();
    }

    void timer(const ros::TimerEvent&)
    {
        calc();
    }

    void calc()
    {
        if (joy.axes.empty())
            return;

        // publish twist

        static bool stopped = true;
        geometry_msgs::Twist twist;
        twist.linear.x = linearConstant * joy.axes[1];
        twist.angular.z = angularConstant * joy.axes[3];

        bool stop = twist.linear.x == 0 && twist.angular.y == 0;

        if (!(stop && stopped))
            pub_twist.publish(twist);
        stopped = stop;


        // publish FORWARD/LEFT/RIGHT

        static std::string prev_cmd = "";
        std::string cmd;
        if (joy.buttons[0] || joy.axes[7] == -1)
            cmd = "STOP";
        else if (joy.buttons[1] || joy.axes[6] == -1)
            cmd = "RIGHT";
        else if (joy.buttons[2] || joy.axes[6] == 1)
            cmd = "LEFT";
        else if (joy.buttons[3] || joy.axes[7] == 1)
            cmd = "FORWARD";

        if (prev_cmd != cmd && cmd != "")
        {
            prev_cmd = cmd;
            std_msgs::String msg_cmd;
            msg_cmd.data = cmd;
            pub_direction.publish(msg_cmd);
        }
    }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "xbox_node");
  Xbox xbox_node;
  xbox_node.run();
}

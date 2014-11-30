#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#define _USE_MATH_DEFINES
#include <cmath>


class PoseNode
{
private:
    ros::NodeHandle nh;
    double baseDiameter;
    double wheelDiameter;
    double ticsPerRevolution;
    double radiansPerTick;
    double distancePerTick;

    double x;
    double y;
    double theta;

    ros::Publisher pub_pose;
    ros::Subscriber sub_encoders;
    ros::Subscriber sub_pose_correction;
    tf::TransformBroadcaster pub_transform;

public:
    PoseNode()
        : nh("~")
        , x(0)
        , y(0)
        , theta(0)
    {
        pub_pose = nh.advertise<nav_msgs::Odometry> ("", 1);
        sub_pose_correction = nh.subscribe ("/wall_brain/pose_correction", 1, &PoseNode::correctionCallback, this);
        sub_encoders = nh.subscribe ("/arduino/encoders", 1, &PoseNode::encoderCallback, this);

        loadParameters();
    }

    void loadParameters()
    {
        nh.getParam("/base/diameter", baseDiameter);
        nh.getParam("/base/wheels/diameter", wheelDiameter);
        nh.getParam("/base/motors/tics_per_revolution", ticsPerRevolution);
        radiansPerTick = (wheelDiameter/baseDiameter) / 2 / 180 * M_PI;
        distancePerTick = M_PI * wheelDiameter / ticsPerRevolution;
    }

    void correctionCallback(const nav_msgs::Odometry &msg)
    {
        theta = tf::getYaw(msg.pose.pose.orientation);
    }

    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
    {
        ros::Time now = ros::Time::now();

        //calculate new pose
        theta += (msg->delta_encoder1 - msg->delta_encoder2)*radiansPerTick;
        float deltaDistance = (msg->delta_encoder1 +msg->delta_encoder2) / 2.0 *distancePerTick;
        x -= deltaDistance*std::cos(theta);
        y -= deltaDistance*std::sin(theta);

        publishOdometry(now);
    }

    void publishOdometry(ros::Time now)
    {
        //publish "robot" transform
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);
        pub_transform.sendTransform(tf::StampedTransform(transform, now, "map", "robot"));

        //publish pose
        nav_msgs::Odometry pose;
        pose.header.stamp = now;
        //TODO: this is what we would like to do, but rviz Odometry marker doesn't work then
        //pose.header.frame_id = "robot";
        //pose.pose.pose.orientation.w = 1;
        pose.header.frame_id = "map";
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.orientation.x = q.x();
        pose.pose.pose.orientation.y = q.y();
        pose.pose.pose.orientation.z = q.z();
        pose.pose.pose.orientation.w = q.w();
        pub_pose.publish(pose);
    }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pose_node");
    PoseNode pose_node;
    pose_node.run();
}

#ifndef WHEATLEY_COMMON_H
#define WHEATLEY_COMMON_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

namespace wheatley
{
    class NiceBaseClass
    {
    protected:
        ros::NodeHandle nh;

        NiceBaseClass()
            : nh("~")
        {}


        template<typename T>
        T requireParameter(const std::string& name)
        {
            if (!nh.hasParam(name))
            {
                ROS_FATAL_STREAM("couldn't find parameter "<<nh.resolveName(name));
                ros::shutdown();
                exit(0);
            }
            T value;
            nh.getParam(name, value);
            ROS_INFO_STREAM("loaded parameter "<<nh.resolveName(name)<<": "<<value);
            return value;
        }
    };

    geometry_msgs::Pose getPose(const tf::TransformListener& tfl, const ros::Time& stamp, const std::string& source_frame, const std::string& target_frame)
    {
        static const tf::Pose pose_identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));
        tf::Stamped<tf::Pose> pose;
        tfl.transformPose(target_frame, tf::Stamped<tf::Pose> (pose_identity, stamp, source_frame), pose);
        geometry_msgs::Pose pose_msg;
        tf::poseTFToMsg(pose, pose_msg);
        return pose_msg;
    }

    template <typename T> T clamp(const T& value, const T& low, const T& high)
    {
      return value < low ? low : (value > high ? high : value);
    }
}

#endif

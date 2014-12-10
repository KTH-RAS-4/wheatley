#include <ros/ros.h>
#include <ras_arduino_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <algorithm>
#include <sound_play/SoundRequest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <sensors/Distance.h>
#include <wheatley_common/common.h>
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/LU>


using std::string;
namespace gu=occupancy_grid_utils;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;

namespace wheatley
{
    class KalmanNode : NiceBaseClass
    {
    private:
        ros::Subscriber sub_map;
        ros::Subscriber sub_ir;
        ros::Subscriber sub_encoders;
        ros::Publisher pub_sim_ir;
        tf::TransformListener tfl;

        tf::StampedTransform tf_front;
        tf::StampedTransform tf_rear;
        tf::StampedTransform tf_left_front;
        tf::StampedTransform tf_left_rear;
        tf::StampedTransform tf_right_front;
        tf::StampedTransform tf_right_rear;

        nm::OccupancyGrid map;

        const string fixed_frame;
        const string robot_frame;

        double MAX_RANGE_SHORT_DISTANCE_SENSOR;
        double MAX_RANGE_LONG_DISTANCE_SENSOR;
        double ROBOT_SENSORS_GAP_DISTANCE;

    public:
        KalmanNode()
            : fixed_frame("map")
            , robot_frame("robot")
        {
            ros::Time now(0);

            while (!tfl.waitForTransform("ir_front", robot_frame, now, ros::Duration(1)))
                ROS_ERROR("Couldn't find transform from 'robot' to 'ir_front', retrying...");

            tfl.lookupTransform(robot_frame, "ir_front",       now, tf_front);
            tfl.lookupTransform(robot_frame, "ir_rear",        now, tf_rear);
            tfl.lookupTransform(robot_frame, "ir_left_front",  now, tf_left_front);
            tfl.lookupTransform(robot_frame, "ir_left_rear",   now, tf_left_rear);
            tfl.lookupTransform(robot_frame, "ir_right_front", now, tf_right_front);
            tfl.lookupTransform(robot_frame, "ir_right_rear",  now, tf_right_rear);

            sub_map = nh.subscribe("map_in", 1, &KalmanNode::callback_map, this);
            sub_ir = nh.subscribe("/sensors/ir/distances", 1, &KalmanNode::callback_ir, this);
            pub_sim_ir = nh.advertise<sensors::Distance>("simulated_ir_distances", 1);

            loadParameters();
        }

        void callback_map(const nm::OccupancyGrid::ConstPtr& msg)
        {
            map = *msg;
        }

        void callback_ir(const sensors::Distance::ConstPtr &realIR)
        {
            gm::Pose pose = getPose(tfl, realIR->header.stamp, robot_frame, fixed_frame);
            sensors::Distance simulatedIR = simulateIrDistances(pose, realIR->header.stamp);
            pub_sim_ir.publish(simulatedIR);
            calc(*realIR, simulatedIR, pose);
        }

        void calc(const sensors::Distance& real, const sensors::Distance& simulated, const gm::Pose& pose)
        {
            static gm::Pose prev_pose;
            double theta = tf::getYaw(pose.orientation);

            double u_x = pose.position.x-prev_pose.position.x;
            double u_y = pose.position.y-prev_pose.position.y;







            prev_pose = pose;
        }

        sensors::Distance simulateIrDistances(const gm::Pose& pose, const ros::Time& stamp)
        {
            tf::Pose tf_pose;
            tf::poseMsgToTF(pose, tf_pose);

            sensors::Distance distance;
            distance.header.stamp = stamp;
            distance.front       = simulateIrSensor(tf_pose * tf_front);
            distance.rear        = simulateIrSensor(tf_pose * tf_rear);
            distance.left_front  = simulateIrSensor(tf_pose * tf_left_front);
            distance.left_rear   = simulateIrSensor(tf_pose * tf_left_rear);
            distance.right_front = simulateIrSensor(tf_pose * tf_right_front);
            distance.right_rear  = simulateIrSensor(tf_pose * tf_right_rear);
            return distance;
        }

        double simulateIrSensor(const tf::Pose& pose)
        {
            static const tf::Vector3 forward(100, 0, 0);
            gm::Point from, towards;
            tf::pointTFToMsg(pose.getOrigin(), from);
            tf::pointTFToMsg(pose*forward, towards);

            BOOST_FOREACH(const gu::Cell& c, gu::rayTrace(map.info, from, towards, true, true))
            {
                gu::index_t ind = gu::cellIndex(map.info, c);
                if (!gu::withinBounds(map.info, c))
                    break;
                if (map.data[ind] == gu::OCCUPIED)
                {
                    tf::Point tf_hit;
                    gm::Point hit = gu::cellCenter(map.info, c);
                    tf::pointMsgToTF(hit, tf_hit);
                    return (tf_hit - pose.getOrigin()).length();
                }
            }
            return -1;
        }

        void loadParameters()
        {
            //nh.getParam("/base/diameter", baseDiameter);

        }

        void run()
        {
            ros::spin();
        }
    };
}

int main (int argc, char **argv)
{

    if (1){
    Eigen::Matrix3d sigma,G,R,I;//[3X3]
    Eigen::Vector3d mu,a;//[3X1]
    Eigen::RowVector3d H,b,c,i,d;//[1X3]

    R(0,0)=1;
    R(0,1)=0;
    R(0,2)=0;
    R(1,0)=0;
    R(1,1)=2;
    R(1,2)=0;
    R(2,0)=0;
    R(2,1)=0;
    R(2,2)=3;

    I(0,0)=1;
    I(0,1)=0;
    I(0,2)=0;
    I(1,0)=0;
    I(1,1)=1;
    I(1,2)=0;
    I(2,0)=0;
    I(2,1)=0;
    I(2,2)=1;

    G=R*R;

    H(0,0)=1;
    H(0,1)=2;
    H(0,2)=3;

    i(0,0)=1;
    i(0,1)=1;
    i(0,2)=1;

    mu(0,0)=10;
    mu(1,0)=20;
    mu(2,0)=30;


    a=R*mu; //[3X1]=[3x3][3x1]
    b=H*R;//[1X3]=(1X3)(3X1)

    double num =H*mu; //[1x1]=[1x3][3x1]

    c=H+(10*i);//sum a constant to a vector



    ROS_INFO("hola %f",G(0,0));
    ROS_INFO("hola %f",G(2,2));
    ROS_INFO("hola %f",b(0,0));
    ROS_INFO("hola %f",b(0,1));
    ROS_INFO("hola %f",b(0,2));
    ROS_INFO("hola %f",a(0,0));
    ROS_INFO("hola %f",a(1,0));
    ROS_INFO("hola %f",a(2,0));
    ROS_INFO("hola %f",num);
    ROS_INFO("hola %f",c(0,0));
    ROS_INFO("hola %f",c(0,1));
    ROS_INFO("hola %f",c(0,2));
    ROS_INFO("hola %f",d(0,0));
    ROS_INFO("hola %f",d(0,1));
    ROS_INFO("hola %f",d(0,2));
    }

    ros::init(argc, argv, "ekf_node");
    wheatley::KalmanNode ekf_node;
    ekf_node.run();
}


/*
MAX_RANGE_LONG_DISTANCE_SENSOR=0.60;
MAX_RANGE_SHORT_DISTANCE_SENSOR=0.20;
ROBOT_SENSORS_GAP_DISTANCE=0.17;

double angle_left, angle_right, angle;

//pending if difference >10 , disregard

if (distance.right_front < MAX_RANGE_SHORT_DISTANCE_SENSOR  && distance.right_rear < MAX_RANGE_SHORT_DISTANCE_SENSOR)
{
    angle_right=atan((distance.right_front-distance.right_rear)/ROBOT_SENSORS_GAP_DISTANCE);
}
else
{
    angle_right=0;
}

if (distance.left_front < MAX_RANGE_SHORT_DISTANCE_SENSOR  && distance.left_rear < MAX_RANGE_SHORT_DISTANCE_SENSOR)
{
    angle_left=-1*atan((distance.left_front-distance.left_rear)/ROBOT_SENSORS_GAP_DISTANCE);
}
else
{
    angle_left=0;
}


if(angle_right!=0)
{
    angle=angle_right;
    if (angle_left!=0)
    {
        angle=(angle_right+angle_left)/2;
    }
}
else if (angle_left!=0)
    angle=angle_left;

ROS_INFO_STREAM("angle= " <<angle);
ROS_INFO("theta= [%f]",theta);
ROS_INFO_STREAM("**********diff= " <<theta-angle);

if(0)
{
    string answer;

    gm::Pose forward_pose;
    forward_distance=robot_outer_diameter;
    forward_pose.position.x = pose.position.x+cos(theta)*(forward_distance+robot_outer_diameter)/2;
    forward_pose.position.y = pose.position.y+sin(theta)*(forward_distance+robot_outer_diameter)/2;
    /// return UNOCCUPIED if all the cells in the square centered at this point with side 2*r contains no obstacles
    /// return OCCUPIED if at least one cell is occupied.
    /// retrun unkown other wise.
    /// ///UNOCCUPIED=0; OCCUPIED=100; UNKNOWN=255; ERROR=33;
    int a = gu::IsWindowFree(&map_oc, forward_pose.position, forward_distance/2);
    switch (a){
        case 0:
           answer="unoccupied";
            break;
        case 100:
            answer="*********occupied";
            break;
        case 255:
            answer="***********unknown";
            break;
        case 33:
            answer="**************ERRROR";
            break;
    }

    ROS_INFO_STREAM("dermineOccupancy = " << answer);

    // gu::determineOccupancy (map_oc.hit_counts, map_oc.pass_through_counts,map_oc.occupancy_threshold,map_oc.min_pass_through);
    // if (answer)
    //    ROS_INFO("free");
    //else
    //  ROS_INFO("******************occupied*********************");

    //void addKnownFreePoint (OverlayClouds* overlay, const gm::Point& p, const double r)
    //void addKnownOccupiedPoint (OverlayClouds* overlay, const gm::Point& p, const double r)
    //bool IsWindowFree (OverlayClouds* overlay, const gm::Point& p, const double r)
}*/

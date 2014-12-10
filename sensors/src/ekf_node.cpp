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

        Eigen::Matrix3f prev_sigma;//[3X3]
        double Q; //5 cm
        double lambda; //chi2inv(0.8,1);
        Eigen::Matrix3f R;//[3X3]

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

            prev_sigma << 0.05,    0, 0,
                             0, 0.05, 0,
                             0,    0, 0.01;//[3X3]
            Q=0.05; //5 cm
            lambda=1.0742; //chi2inv(0.8,1);

            R << 0.05,    0, 0,
                    0, 0.05, 0,
                    0,    0, 0.01;//[3X3]

            loadParameters();
        }

        void callback_map(const nm::OccupancyGrid::ConstPtr& msg)
        {
            map = *msg;
        }

        void callback_ir(const sensors::Distance::ConstPtr &realIR)
        {
            if (!tfl.waitForTransform(fixed_frame, robot_frame, realIR->header.stamp, ros::Duration(0.2)))
            {
                ROS_ERROR_STREAM("timeout waiting for transform from "<<robot_frame<<" to "<<fixed_frame);
                return;
            }

            gm::Pose pose = getPose(tfl, realIR->header.stamp, robot_frame, fixed_frame);
            sensors::Distance simulatedIR = simulateIrDistances(pose, realIR->header.stamp);
            pub_sim_ir.publish(simulatedIR);

            static bool first_pose_callback = true;
            static gm::Pose prev_pose;

            if (!first_pose_callback)
                calc(*realIR, simulatedIR, pose, prev_pose);

            prev_pose = pose;
            first_pose_callback = false;
        }

        void calc(const sensors::Distance& real, const sensors::Distance& simulated, const gm::Pose& pose, const gm::Pose& prev_pose)
        {
            double theta = tf::getYaw(pose.orientation);
            double u_x = pose.position.x-prev_pose.position.x;
            double u_y = pose.position.y-prev_pose.position.y;

            //prev_sigma [3x3]

            Eigen::VectorXd inliers(6);//[6X1]
            Eigen::VectorXf nu(6);//[6X1]
            Eigen::MatrixXf H_buffer(6,3) ;//[6X3]
            Eigen::Matrix3f sigma,G;//[3X3]

            G<< 1,0,-u_y,
                0,1, u_x,
                0,0,  1;

            sigma=G*prev_sigma*G.transpose()+R;



            int i;
            double x,y;
            for (i=0;i<6;i++){

                H_buffer(i,0)=x;
                H_buffer(i,1)=y;
                H_buffer(i,2)=0;
            }







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
            return -0.1;
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
/*
    if (0){
    Eigen::Matrix3d sigma,G,R,I,k,u;//[3X3]
    Eigen::Vector3d mu,a,u1,u2,u3,u4;//[3X1]
    Eigen::RowVector3d H,b,c,i,d;//[1X3]

    int j=4;
    Eigen::MatrixXd n(3,j) ;//[3Xj]
    Eigen::MatrixXd n_t(j,3) ;//[jX3]

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

    H(0,0)=1;
    H(0,1)=2;
    H(0,2)=3;

    i(0,0)=1;
    i(0,1)=1;
    i(0,2)=1;

    mu(0,0)=10;
    mu(1,0)=20;
    mu(2,0)=30;

    u1<<1,2,3;
    u2<<11,22,33;
    u3<<111,222,333;
    u4<<1111,2222,3333;

    n << u1,u2,u3,u4;
    n_t=n.transpose();

    //////////////////////////////

    G=R*R;
    b=H*R;//[1X3]=(1X3)(3X1)
    a=R*mu; //[3X1]=[3x3][3x1]

    double num =H*mu; //[1x1]=[1x3][3x1]

    c=H+(10*i);//sum a constant to a vector

    d=mu.transpose();
    sigma=R.inverse();

     k <<1,2,3,4,5,6,7,8,9;

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
    ROS_INFO("hola %f",sigma(0,0));
    ROS_INFO("hola %f",sigma(1,1));
    ROS_INFO("hola %f",sigma(2,2));
    ROS_INFO("hola %f",k(0,0));
    ROS_INFO("hola %f",k(1,1));
    ROS_INFO("hola %f",k(2,2));

    ROS_INFO("hola %f",u1(0,0));
    ROS_INFO("hola %f",u1(1,0));
    ROS_INFO("hola %f",u1(2,0));
    ROS_INFO("hola %f",u2(0,0));
    ROS_INFO("hola %f",u2(1,0));
    ROS_INFO("hola %f",u2(2,0));
    ROS_INFO("hola %f",u3(0,0));
    ROS_INFO("hola %f",u3(1,0));
    ROS_INFO("hola %f",u3(2,0));
    ROS_INFO("hola %f",u4(0,0));
    ROS_INFO("hola %f",u4(1,0));
    ROS_INFO("hola %f",u4(2,0));

//    ROS_INFO("\nhola %f",u(0,1));
//    ROS_INFO("hola %f",u(0,2));
//    ROS_INFO("hola %f",u(0,0));

    ROS_INFO_STREAM("n = " << n);

    ROS_INFO_STREAM("n.transpose() = \n" << n_t);
    ROS_INFO_STREAM("n.transpose() = \n" << n_t(1,1));


    }

        Eigen::VectorXf diff(6);


    if(1){
        Eigen::VectorXf diff(6);
        Eigen::VectorXd inliers(6);
        diff<<1,2,3,4,5,6;
        ROS_INFO_STREAM("diff = \n" << diff);
        ROS_INFO_STREAM("diff(5) = " << diff(5));


        Eigen::MatrixXf H_buffer(6,3) ;//[6X3]
        double x,y;
        x=1;
        y=11;
        for (int hhh=0;hhh<6;hhh++){
            H_buffer(hhh,0)=x*hhh;
            H_buffer(hhh,1)=y*hhh;
            H_buffer(hhh,2)=0;
        }

        ROS_INFO_STREAM("H_buffer = \n" << H_buffer);
        ROS_INFO_STREAM("H_buffe(5,2) = " << H_buffer(5,1));

        double num_inliers=0;
        for (int hhh=0;hhh<6;hhh++){
            H_buffer(hhh,0)=x*hhh;
            H_buffer(hhh,1)=y*hhh;
            H_buffer(hhh,2)=0;
        }


        int result;
        for (int o=0;o<6;o++){

            ROS_INFO_STREAM("diff(5) = " << diff(o));
            result=diff(o);
            if (result%2)
                inliers(o)=0;
            else
            {
                inliers(o)=1;
                num_inliers++;
            }

        }
        ROS_INFO_STREAM("inliers = \n" << inliers);
        Eigen::VectorXf nu(num_inliers);
        Eigen::MatrixXf H_hat(num_inliers,3);
        int oo=0;
        for ( int o=0;o<6;o++)
        {
            if(inliers(o))
            {
                nu(oo)=diff(o);
                H_hat(oo,0)=H_buffer(o,0);
                H_hat(oo,1)=H_buffer(o,1);
                H_hat(oo,2)=H_buffer(o,2);
                oo++;
            }
        }
        ROS_INFO_STREAM("H_hat = \n" << H_hat);
        ROS_INFO_STREAM("nu = \n" << nu);

        Eigen::Vector3f pose1;
        pose1=H_hat*nu;
        ROS_INFO_STREAM("pose = \n" << pose1);


    }
    */


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

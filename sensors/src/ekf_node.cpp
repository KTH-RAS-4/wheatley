#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/tf.h>
#include <algorithm>
#include <sound_play/SoundRequest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <map>
#include <sensors/Distance.h>
#include <wheatley_common/common.h>


namespace gu=occupancy_grid_utils;
using std::string;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;

namespace wheatley
{
    class KalmanNode : NiceBaseClass
    {
    private:
        ros::Subscriber sub_pose;
        ros::Subscriber sub_map;
        ros::Subscriber sub_ir;
        ros::WallTimer timer;

        gu::OverlayClouds map_oc;
        nm::OccupancyGrid map_og;
        sensors::Distance distance;

        string fixed_frame;
        gm::Pose pose;
        double robot_outer_diameter;
        double forward_distance;
        bool got_pose;

        double MAX_RANGE_SHORT_DISTANCE_SENSOR;
        double MAX_RANGE_LONG_DISTANCE_SENSOR;
        double ROBOT_SENSORS_GAP_DISTANCE;

    public:
        KalmanNode()
            : fixed_frame("map")
            , robot_outer_diameter(requireParameter<double>("/base/outer_diameter"))
            , got_pose(false)
        {
            MAX_RANGE_LONG_DISTANCE_SENSOR=0.60;
            MAX_RANGE_SHORT_DISTANCE_SENSOR=0.20;
            ROBOT_SENSORS_GAP_DISTANCE=0.17;

            double rate = requireParameter<double>("rate");

            sub_pose = nh.subscribe("/sensors/pose", 1000, &KalmanNode::callback_pose, this);
            sub_map = nh.subscribe("map_in", 1, &KalmanNode::callback_map, this);
            sub_ir = nh.subscribe("/sensors/ir/distances", 1000, &KalmanNode::callback_ir, this);
            timer = nh.createWallTimer(ros::WallDuration(1/rate), &KalmanNode::callback_timer, this);
        }

        void callback_map(const nm::OccupancyGrid::ConstPtr& msg)
        {
            /*
            inflated_map = *gu::inflateObstacles(map, robot_diameter/2);
            inflated_map.header.frame_id = msg->header.frame_id;
            inflated_map.header.stamp = msg->header.stamp;
            pub_map.publish(inflated_map);
            */
            //map_og = *msg;
            //map_oc = gu::createCloudOverlay(map_og, fixed_frame, 0.1, 10, 1);
            map_oc = gu::createCloudOverlay(*msg, fixed_frame, 0.1, 10, 1);
            //nm::OccupancyGrid::ConstPtr grid = gu::getGrid(map_oc);
        }

        void callback_pose(const nm::Odometry::ConstPtr &msg)
        {
            pose = msg->pose.pose;
            got_pose = true;
        }

        void callback_ir(const sensors::Distance::ConstPtr &msg)
        {
            if (!got_pose)
            {
                ROS_ERROR("callback_ir(): got no pose yet");
                return;
            }

            double theta = tf::getYaw(pose.orientation);
            distance = *msg;
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

            calc();
        }

        void callback_timer(const ros::WallTimerEvent&)
        {
            //hmm, only run when we have new ir measurements
            //calc();
        }

        void calc()
        {
            if (!got_pose)
            {
                ROS_ERROR("calc(): got no pose yet");
                return;
            }

            double theta = tf::getYaw(pose.orientation);
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
            }
        }

        void run()
        {
            ros::spin();
        }
    };
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ekf_node");
    wheatley::KalmanNode ekf_node;
    ekf_node.run();
}

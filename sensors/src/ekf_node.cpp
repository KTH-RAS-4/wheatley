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
#include <tf/transform_broadcaster.h>


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
        ros::Publisher pub_pose_ekf;
        tf::TransformBroadcaster pub_transform;

        tf::StampedTransform tf_front;
        tf::StampedTransform tf_rear;
        tf::StampedTransform tf_left_front;
        tf::StampedTransform tf_left_rear;
        tf::StampedTransform tf_right_front;
        tf::StampedTransform tf_right_rear;
        tf::TransformListener tfl;

        nm::OccupancyGrid map;

        const string fixed_frame;
        const string robot_frame;

        Eigen::Vector3f prev_mu; //[3X1]
        Eigen::Matrix3f prev_sigma;//[3X3]
        Eigen::Matrix3f R;//[3X3]
        double lambda,Q,threshold, SHORT_DIS_LIM,  LONG_DIS_LIM;
        int static_update_lim;

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
            pub_pose_ekf = nh.advertise<nav_msgs::Odometry> ("pose_ekf", 1);

            loadParameters();

        }
        void loadParameters()
        {           
            double r1     = requireParameter<double>("r1");
            double r2     = requireParameter<double>("r2");
            double r3     = requireParameter<double>("r3");
            double mu1    = requireParameter<double>("mu1");
            double mu2    = requireParameter<double>("mu2");
            double mu3    = requireParameter<double>("mu3");

            //measurement noise
            Q      = requireParameter<double>("Q");

            //outliers threshold
            lambda = requireParameter<double>("lambda");

            //proces noice
            R <<   r1,    0,  0,
                    0,   r2,  0,
                    0,    0, r3;//[3X3]

            //intial position
            prev_mu<< mu1,
                      mu2,
                      mu3;//[3X1]

            //intial sigma Covariance matrix
            prev_sigma << 0.05,    0, 0,
                             0, 0.05, 0,
                            0,    0, 0.01;//[3X3]
            //measurement noise
            static_update_lim    = requireParameter<double>("static_update_lim");

            //measurement noise
            threshold    = requireParameter<double>("threshold");

            //max distance acetable for considere sensor reading and simulation valid
            SHORT_DIS_LIM    = requireParameter<double>("SHORT_DIS_LIM");
            LONG_DIS_LIM    = requireParameter<double>("LONG_DIS_LIM");


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
                 ROS_INFO_STREAM("call back error");
                return;
            }

            gm::Pose pose = getPose(tfl, realIR->header.stamp, robot_frame, fixed_frame);
            sensors::Distance simulatedIR = simulateIrDistances(pose, realIR->header.stamp);
            pub_sim_ir.publish(simulatedIR);

            static bool first_pose_callback = true;
            static gm::Pose prev_pose;

            if (!first_pose_callback)
                calc_ekf(*realIR, simulatedIR, pose, prev_pose);

            prev_pose = pose;
            first_pose_callback = false;           
        }

        void calc_ekf(const sensors::Distance& real, const sensors::Distance& simulated, const gm::Pose& pose, const gm::Pose& prev_pose)
        {
            ros::Time now = ros::Time::now();

            //do not ekf if the robot is static
            int static static_update=0;

            //get the Ut
            double     u_x = pose.position.x-prev_pose.position.x;
            double     u_y = pose.position.y-prev_pose.position.y;
            double u_theta = tf::getYaw(pose.orientation) - tf::getYaw(prev_pose.orientation);

            std::cout << "u_x: "    << std::abs(u_x)     << std::endl;
            std::cout << "u_y: "    << std::abs(u_y)     << std::endl;
            std::cout << "u_theta: "<< std::abs(u_theta) << std::endl;

            bool odometryUpdateTrue = std::abs(u_x)>threshold || std::abs(u_y)>threshold || std::abs(u_theta)>threshold;

            std::cout << "static_update: "<< static_update<< std::endl;

            std::cout << "threshold: "<<threshold << std::endl;

            if (!odometryUpdateTrue)  {  static_update++;  }
            else                      {  static_update=0;  }

            if (static_update<static_update_lim ){

                    int NUM_MEASUREMENTS=6;

                    //initialize matrix and vectors
                    Eigen::VectorXd inliers(NUM_MEASUREMENTS), outlier(NUM_MEASUREMENTS);//[6X1]
                    Eigen::VectorXf nu_buffer(NUM_MEASUREMENTS);//[6X1]
                    Eigen::MatrixXf H_buffer(NUM_MEASUREMENTS,3) ;//[6X3]
                    Eigen::Matrix3f sigma,sigma_hat,G;//[3X3]
                    Eigen::Vector3f mu, mu_hat,update; //[3X1]

                    //prediction mu
                    mu_hat(0,0)=prev_mu(0,0)+u_x;
                    mu_hat(1,0)=prev_mu(1,0)+u_y;
                    mu_hat(2,0)=prev_mu(2,0)+u_theta;
                    double theta = mu_hat(2,0);

                    gm::Pose pose_hat=pose;
                    pose_hat.position.x=mu_hat(0,0);
                    pose_hat.position.y=mu_hat(1,0);
                    //pose_hat.orientation=mu_hat(2,0);
                    sensors::Distance simulated2 = simulateIrDistances(pose_hat, now);
                    //pub_sim_ir.publish(simulated);


                    //jacobian
                    G<< 1,0,-u_y,
                        0,1, u_x,
                        0,0,  1;

                     std::cout<<"G \n"<< G <<std::endl;

                    //predicted sigma (covarianze matrix)
                    if (odometryUpdateTrue)
                        sigma_hat=G*prev_sigma*G.transpose()+R;
                    else
                        sigma_hat=prev_sigma;


                    //measurement model
                    //nav_msgs::Odometry pose_hat;
                    //pose_hat=get_pose_ekf(now, mu_hat(0,0), mu_hat(1,0), mu_hat(2,0));
                    sensors::Distance simulated3 = simulateIrDistances(pose, now);

                    //jacobian and nu for all the measuremts.
                    //data association is implicit in simulated (measuremt model)
                    std::cout<<"\nreal.front     "<< real.front <<std::endl;
                    std::cout<<"simulated.front"<< simulated.front <<std::endl;

                    H_buffer(0,0)=real.front*cos(theta)/simulated.front;
                    H_buffer(0,1)=real.front*sin(theta)/simulated.front;
                    H_buffer(0,2)=0;
                    nu_buffer(0)=real.front-simulated.front;
                    if (0<=real.front && real.front<= LONG_DIS_LIM && 0<=simulated.front && simulated.front<= LONG_DIS_LIM)
                        outlier(0)=0;
                    else {
                        outlier(0)=1;
                        std::cout<<"outlier" <<std::endl;
                    }



                    std::cout<<"\nreal.rear     "<< real.rear <<std::endl;
                    std::cout<<"simulated.rear"<< simulated.rear <<std::endl;

                    H_buffer(1,0)=real.rear*cos(theta)/simulated.rear;
                    H_buffer(1,1)=real.rear*sin(theta)/simulated.rear;
                    H_buffer(1,2)=0;
                    nu_buffer(1)=real.rear-simulated.rear;
                    if (0<=real.rear && real.rear<= LONG_DIS_LIM && 0<=simulated.rear && simulated.rear<= LONG_DIS_LIM)
                        outlier(1)=1;//always outlier, until sensor is replaced
                    else
                    {
                        outlier(1)=1;
                        std::cout<<"outlier" <<std::endl;
                    }

                    std::cout<<"\nreal.left_front     "<< real.left_front <<std::endl;
                    std::cout<<"simulated.left_front"<< simulated.left_front <<std::endl;

                    H_buffer(2,0)=real.left_front*cos(theta)/simulated.left_front;
                    H_buffer(2,1)=real.left_front*sin(theta)/simulated.left_front;
                    H_buffer(2,2)=0;
                    nu_buffer(2)=real.left_front-simulated.left_front;
                    if (0<=real.left_front && real.left_front<= SHORT_DIS_LIM && 0<=simulated.left_front && simulated.left_front<= SHORT_DIS_LIM)
                        outlier(2)=0;
                    else
                    {
                        outlier(2)=1;
                        std::cout<<"outlier" <<std::endl;
                    }

                    std::cout<<"\nreal.left_rear     "<< real.left_rear <<std::endl;
                    std::cout<<"simulated.left_rear"<< simulated.left_rear <<std::endl;

                    H_buffer(3,0)=real.left_rear*cos(theta)/simulated.left_rear;
                    H_buffer(3,1)=real.left_rear*sin(theta)/simulated.left_rear;
                    H_buffer(3,2)=0;
                    nu_buffer(3)=real.left_rear-simulated.left_rear;
                    if (0<=real.left_rear && real.left_rear<= SHORT_DIS_LIM && 0<=simulated.left_rear && simulated.left_rear<= SHORT_DIS_LIM)
                        outlier(3)=0;
                    else
                    {
                        outlier(3)=1;
                        std::cout<<"outlier" <<std::endl;
                    }


                    std::cout<<"\nreal.right_front     "<< real.right_front <<std::endl;
                    std::cout<<"simulated.right_front"<< simulated.right_front <<std::endl;

                    H_buffer(4,0)=real.right_front*cos(theta)/simulated.right_front;
                    H_buffer(4,1)=real.right_front*sin(theta)/simulated.right_front;
                    H_buffer(4,2)=0;
                    nu_buffer(4)=real.right_front-simulated.right_front;
                    if (0<=real.right_front && real.right_front<= SHORT_DIS_LIM && 0<=simulated.right_front && simulated.right_front<= SHORT_DIS_LIM)
                        outlier(4)=0;
                    else
                    {
                        outlier(4)=1;
                        std::cout<<"outlier" <<std::endl;
                    }

                    std::cout<<"\nreal.right_rear     "<< real.right_rear <<std::endl;
                    std::cout<<"simulated.right_rear"<< simulated.right_rear <<std::endl;

                    H_buffer(5,0)=real.right_rear*cos(theta)/simulated.right_rear;
                    H_buffer(5,1)=real.right_rear*sin(theta)/simulated.right_rear;
                    H_buffer(5,2)=0;
                    nu_buffer(5)=real.right_rear-simulated.right_rear;
                    if (0<=real.right_rear && real.right_rear<= SHORT_DIS_LIM && 0<=simulated.right_rear && simulated.right_rear<= SHORT_DIS_LIM)
                        outlier(5)=0;
                    else
                    {
                        outlier(5)=1;
                        std::cout<<"outlier" <<std::endl;
                    }

                    //detect outliers
                    int num_inliers=0;
                    Eigen::RowVector3f hi; //[1x3]
                    double S,D;
                    for(int i=0;i<NUM_MEASUREMENTS;i++){
                         hi<< H_buffer(i,0), H_buffer(i,1), H_buffer(i,2);
                         S=hi*sigma_hat*hi.transpose()+Q;
                         D=nu_buffer(i)*nu_buffer(i)/S;
                         if (D>=lambda ||  outlier(i))
                         {
                            inliers(i)=0;
                         }
                         else
                         {
                            inliers(i)=1;
                            num_inliers++;
                            std::cout<<"**************************inlier num = "<< i <<std::endl;
                            std::cout<<"nu("<<i <<") = "<< nu_buffer(i) <<std::endl;
                         }
                    }

                    //compute the H and nu for inliers.
                    Eigen::VectorXf nu(num_inliers);
                    Eigen::MatrixXf H_hat(num_inliers,3);
                    int j=0;
                    for ( int i=0;i<NUM_MEASUREMENTS;i++)
                    {
                        if(inliers(i))
                        {
                            nu(j)=nu_buffer(i);
                            H_hat(j,0)=H_buffer(i,0);
                            H_hat(j,1)=H_buffer(i,1);
                            H_hat(j,2)=H_buffer(i,2);
                            j++;
                        }
                    }

                   //compute K kalman gain
                   Eigen::MatrixXf I(3,3);
                   I = Eigen::MatrixXf::Identity(3,3);

                   Eigen::MatrixXf S_hat(num_inliers,num_inliers);
                   Eigen::MatrixXf Q_hat(num_inliers,num_inliers);
                   Q_hat = Q * Eigen::MatrixXf::Identity(num_inliers,num_inliers);

                   S_hat=H_hat*sigma_hat*H_hat.transpose()+Q_hat;
                   Eigen::MatrixXf K(3,num_inliers);

                   K=sigma_hat*H_hat.transpose()*S_hat.inverse();

                   //compute the measurement update
                   update=K*nu;

                   //update mu
                   mu=mu_hat+update;
                   prev_mu=mu;//saving for next iteration

                   //update sigma
                   sigma=(I-K*H_hat)*sigma_hat;
                   //store sigma for the next iteration
                   prev_sigma=sigma;

                   // debug  lines
                   //prev_mu=mu_hat; //remove this line
                   std::cout<<" sigma:   \n"<<    sigma <<std::endl;
                   std::cout<<"     H:       "<<    H_hat <<std::endl;
                   std::cout<<"    nu:       "<<    nu <<std::endl;
                   std::cout<<"update:    "<<update.transpose() <<std::endl;
            }// end ekf

           //publish mu
           nav_msgs::Odometry pose_ekf;
           pose_ekf=get_pose_ekf(now, prev_mu(0,0), prev_mu(1,0), prev_mu(2,0));
           pub_pose_ekf.publish(pose_ekf);
           std::cout<<"                                                       mu:       "<<    prev_mu.transpose() <<std::endl;
        }// end function calc_ekf

        nav_msgs::Odometry get_pose_ekf(ros::Time now,double x,double y,double theta)
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
        pub_pose_ekf.publish(pose);
        return pose;
    }

        sensors::Distance simulateIrDistances(const gm::Pose& pose, const ros::Time& stamp)
        {
            tf::Pose tf_pose;
            tf::poseMsgToTF(pose, tf_pose);

            sensors::Distance distance;
            distance.header.stamp = stamp;
            distance.front        = simulateIrSensor(tf_pose * tf_front);
            distance.rear         = simulateIrSensor(tf_pose * tf_rear);
            distance.left_front   = simulateIrSensor(tf_pose * tf_left_front);
            distance.left_rear    = simulateIrSensor(tf_pose * tf_left_rear);
            distance.right_front  = simulateIrSensor(tf_pose * tf_right_front);
            distance.right_rear   = simulateIrSensor(tf_pose * tf_right_rear);
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

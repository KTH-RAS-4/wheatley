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
        double lambda,Q,threshold;
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

        }

        void callback_map(const nm::OccupancyGrid::ConstPtr& msg)
        {
            map = *msg;          
             ROS_INFO_STREAM("call back map");
        }

        void callback_ir(const sensors::Distance::ConstPtr &realIR)
        {
             ROS_INFO_STREAM("calbak ir1");
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
             ROS_INFO_STREAM("call back ir final");
        }

        void calc_ekf(const sensors::Distance& real, const sensors::Distance& simulated, const gm::Pose& pose, const gm::Pose& prev_pose)
        {
             ROS_INFO_STREAM("calc_ekf");
            ros::Time now = ros::Time::now();

            //get the Ut
            double     u_x = pose.position.x-prev_pose.position.x;
            double     u_y = pose.position.y-prev_pose.position.y;
            double u_theta = tf::getYaw(pose.orientation) - tf::getYaw(prev_pose.orientation);

            std::cout << "u_x: "<< abs(u_x) << std::endl;
            std::cout << "u_y: "<< abs(u_y) << std::endl;
            std::cout << "u_theta: "<< abs(u_theta) << std::endl;



            //do not ekf if the robot is static
            int static static_update=0;

            std::cout << "                                                           static_update: "<< static_update<< std::endl;
            std::cout << "                                                           threshold: "<<threshold << std::endl;

            if (abs(u_x)<threshold && abs(u_y)<threshold && abs(u_theta)<threshold)
            {
                static_update++;
            }

            if (abs(u_x)>threshold || abs(u_y)>threshold || abs(u_theta)>threshold)
            {static_update=0;}

            if (abs(u_x)>threshold || abs(u_y)>threshold || abs(u_theta)>threshold || static_update<static_update_lim)


            {
                    int NUM_MEASUREMENTS=6;

                    //initialize matrix and vectors
                    Eigen::VectorXd inliers(NUM_MEASUREMENTS);//[6X1]
                    Eigen::VectorXf nu_buffer(NUM_MEASUREMENTS);//[6X1]
                    Eigen::MatrixXf H_buffer(NUM_MEASUREMENTS,3) ;//[6X3]
                    Eigen::Matrix3f sigma,sigma_hat,G;//[3X3]
                    Eigen::Vector3f mu, mu_hat,update; //[3X1]

                    //prediction mu
                    mu_hat(0,0)=prev_mu(0,0)+u_x;
                    mu_hat(1,0)=prev_mu(1,0)+u_y;
                    mu_hat(2,0)=prev_mu(2,0)+u_theta;
                    double theta = mu_hat(2,0);

                    //jacobian
                    G<< 1,0,-u_y,
                        0,1, u_x,
                        0,0,  1;

                     std::cout<<"G \n"<< G <<std::endl;

                    //predicted sigma (covarianze matrix)
                    sigma_hat=G*prev_sigma*G.transpose()+R;

                    std::cout<<"Sigma_hat \n"<< sigma_hat <<std::endl;

                    //jacobian and nu for all the measuremts.
                    //data association is implicit in simulated (measuremt model)
                    H_buffer(0,0)=real.front*cos(theta)/simulated.front;
                    H_buffer(0,1)=real.front*sin(theta)/simulated.front;
                    H_buffer(0,2)=0;
                    nu_buffer(0)=real.front-simulated.front;

                    H_buffer(1,0)=real.rear*cos(theta)/simulated.rear;
                    H_buffer(1,1)=real.rear*sin(theta)/simulated.rear;
                    H_buffer(1,2)=0;
                    nu_buffer(1)=real.rear-simulated.rear;

                    H_buffer(2,0)=real.left_front*cos(theta)/simulated.left_front;
                    H_buffer(2,1)=real.left_front*sin(theta)/simulated.left_front;
                    H_buffer(2,2)=0;
                    nu_buffer(2)=real.left_front-simulated.left_front;

                    H_buffer(3,0)=real.left_rear*cos(theta)/simulated.left_rear;
                    H_buffer(3,1)=real.left_rear*sin(theta)/simulated.left_rear;
                    H_buffer(3,2)=0;
                    nu_buffer(3)=real.left_rear-simulated.left_rear;

                    H_buffer(4,0)=real.right_front*cos(theta)/simulated.right_front;
                    H_buffer(4,1)=real.right_front*sin(theta)/simulated.right_front;
                    H_buffer(4,2)=0;
                    nu_buffer(4)=real.right_front-simulated.right_front;

                    H_buffer(5,0)=real.right_rear*cos(theta)/simulated.right_rear;
                    H_buffer(5,1)=real.right_rear*sin(theta)/simulated.right_rear;
                    H_buffer(5,2)=0;
                    nu_buffer(5)=real.right_rear-simulated.right_rear;

                    //detect outliers
                    int num_inliers=0;
                    Eigen::RowVector3f hi; //[1x3]
                    double S,D;
                    for(int i=0;i<NUM_MEASUREMENTS;i++){
                         hi<< H_buffer(i,0), H_buffer(i,1), H_buffer(i,2);
                         S=hi*sigma_hat*hi.transpose()+Q;
                         D=nu_buffer(i)*nu_buffer(i)/S;
                         if (D>=lambda)
                         {
                            inliers(i)=1;
                            num_inliers++;
                            std::cout<<"inlier " <<std::endl;
                         }
                         else
                            inliers(i)=0;
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
                   std::cout<<"update:   "<<update.transpose() <<std::endl;
                   //store sigma for the next iteration
                   prev_sigma=sigma;

            }// end ekf
            else static_update++;


           //poblish mu
           publish_pose_ekf(now, prev_mu(0,0), prev_mu(1,0), prev_mu(2,0));
           std::cout<<"                                                       mu:       "<<    prev_mu.transpose() <<std::endl;
        }// end function calc_ekf

        void publish_pose_ekf(ros::Time now,double x,double y,double theta)
    {
         ROS_INFO_STREAM("publish ekf pose");
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
     ROS_INFO_STREAM("main 1");
    ros::init(argc, argv, "ekf_node");
    wheatley::KalmanNode ekf_node;
     ROS_INFO_STREAM("main 2");
    ekf_node.run();
     ROS_INFO_STREAM("main final");
}

/*
    if(1)
    {
        int dim=5;
        Eigen::MatrixXf I(dim,dim);
        I = 1.3 * Eigen::MatrixXf::Identity(dim,dim);
        ROS_INFO_STREAM("I*13= \n" << I);
    }

    if(1){
        Eigen::VectorXf diff(6);
        Eigen::VectorXd inliers(6);
        diff<<1,2,3,4,5,6;
        ROS_INFO_STREAM("diff = \n" << diff);
        ROS_INFO_STREAM("diff(5) = " << diff(5));


        Eigen::MatrixXf H_buffer1(6,3) ;//[6X3]
        double x,y;
        x=1;
        y=11;
        for (int hhh=0;hhh<6;hhh++){
            H_buffer1(hhh,0)=x*hhh;
            H_buffer1(hhh,1)=y*hhh;
            H_buffer1(hhh,2)=0;
        }

        ROS_INFO_STREAM("H_buffer1 = \n" << H_buffer1);
        ROS_INFO_STREAM("H_buffe(5,2) = " << H_buffer1(5,1));

        double num_inliers=0;
        for (int hhh=0;hhh<6;hhh++){
            H_buffer1(hhh,0)=x*hhh;
            H_buffer1(hhh,1)=y*hhh;
            H_buffer1(hhh,2)=0;
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
                H_hat(oo,0)=H_buffer1(o,0);
                H_hat(oo,1)=H_buffer1(o,1);
                H_hat(oo,2)=H_buffer1(o,2);
                oo++;
            }
        }
        ROS_INFO_STREAM("H_hat = \n" << H_hat);
        ROS_INFO_STREAM("nu = \n" << nu);

        Eigen::Vector3f pose1,h1t;//[3x1]
        pose1=H_hat*nu;
        ROS_INFO_STREAM("pose = \n" << pose1);

        Eigen::RowVector3f h1; //[1x3]
        h1<< H_buffer1(4,0), H_buffer1(4,1), H_buffer1(4,2);
        ROS_INFO_STREAM("h1 = " << h1);

        double Q1; //5 cm
        //double lambda; //chi2inv(0.8,1);
        Eigen::Matrix3f R1;//[3X3]

        Q1=0.05; //5 cm
        //lambda=1.0742; //chi2inv(0.8,1);

        R1 << 0.05,    0, 0,
                0, 0.05, 0,
                0,    0, 0.01;//[3X3]

        //h1t=h1.transpose();

        double pose2=h1*R1*h1.transpose()+Q1;

        ROS_INFO_STREAM("result = " << pose2);


    }

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
*/

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

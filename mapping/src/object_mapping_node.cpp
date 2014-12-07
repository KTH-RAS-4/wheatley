#include <iostream>
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <list>
#include <fstream>

#include <vision_msgs/PreprocessedClouds.h>
#include <vision_msgs/Objects.h>
#include <vision_msgs/Object.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ras_arduino_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <map>

#include <sensors/SensorClouds.h>

#include <tf/transform_listener.h>

#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/file.h>
#include <ros/wall_timer.h>

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <wheatley_common/common.h>

namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace nm=nav_msgs;
namespace wheatley {



using std::vector;
using std::string;
using std::list;
using std::abs;




class MapNode : NiceBaseClass{
private:
    ros::Subscriber sub_objects;
    ros::NodeHandle nh;
    /****************************************
     * Associated objects
     ****************************************/
    ros::Publisher rviz_object_pub_;
    ros::Publisher object_pub_;
    boost::mutex mutex_;
    std::string state;
    /****************************************
     * Stores
     ****************************************/

    list<vision_msgs::Object> object_collector;


    int current_iteration;


public:
    MapNode(const ros::NodeHandle& nh_)
        : nh(nh_)
        , state(requireParameter<std::string>("run_state"))
    {
        rviz_object_pub_ = nh.advertise<visualization_msgs::MarkerArray>("map_objects", 100);
        object_pub_ = nh.advertise<vision_msgs::Object>("/object/placement", 100);

        sub_objects = nh.subscribe("/object_recognition/objects", 100, &MapNode::insertSubObject, this);


    }


    void echoObjects() {
        visualization_msgs::MarkerArray object_marker;

        int counter = 0;
        for (list<vision_msgs::Object>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {
            visualization_msgs::Marker marker;
            visualization_msgs::Marker text;
            marker.header.frame_id = text.header.frame_id = "/map";
            marker.ns = text.ns = "objects";
            marker.action = text.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = text.pose.orientation.w = 1.0;

            marker.id = counter++;
            text.id = counter++;

            marker.type = visualization_msgs::Marker::POINTS;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;

            text.scale.z = 0.5;

            marker.color.r = it->r / 255;
            marker.color.g = it->g / 255;
            marker.color.b = it->b / 255;
            marker.color.a = 1.0;

            geometry_msgs::Point p;
            p.x = text.pose.position.x = it->x;
            p.y = text.pose.position.y = it->y;
            p.z = text.pose.position.z = it->z;
            marker.points.push_back(p);
            text.text = it->type;
            object_marker.markers.push_back(marker);
            object_marker.markers.push_back(text);
        }
        rviz_object_pub_.publish(object_marker);
    }

    void insertFileObject(const vision_msgs::Object &new_object) {
        ROS_INFO("Inserting object");
        for (list<vision_msgs::Object>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {

            //If already inserted
            if(abs(new_object.x - it->x) < 0.03 && abs(new_object.y - it->y) < 0.03 && abs(new_object.z - it->z) < 0.03) {
                ROS_INFO("Already there, differences: %f, %f, %f", abs(new_object.x - it->x), abs(new_object.y - it->y), abs(new_object.z - it->z));
                return;
            }
        }
        ROS_INFO("Inserted new object");
        object_collector.push_back(new_object);

        echoObjects();


    }

    void write2File(const vision_msgs::Object _msg)
    {
        std::ofstream ofs("/home/ras/catkin_ws/src/wheatley/objects.txt", std::ios::out | std::ios::app);
        if (ofs.is_open())
        {
          ofs << _msg;
          ofs.close();
        }
    }

    void readFromFile()
    {
        std::ifstream ifs("/home/ras/catkin_ws/src/wheatley/objects.txt", std::ios::in);
        if (ifs.is_open())
         {
            string line;
            int count = 0;
            int l;
            vision_msgs::Object obj;
            while ( getline (ifs,line) )
            {
                if (count == 0)
                {
                    l = line.length();
                    obj.r = atof(line.substr(3,l-3).c_str());
                } else if (count == 1)
                {
                    l = line.length();
                    obj.g = atof(line.substr(3,l-3).c_str());
                } else if (count == 2)
                {
                    l = line.length();
                    obj.b = atof(line.substr(3,l-3).c_str());
                } else if (count == 3)
                {
                    l = line.length();
                    obj.x = atof(line.substr(3,l-3).c_str());
                } else if (count == 4)
                {
                    l = line.length();
                    obj.y = atof(line.substr(3,l-3).c_str());
                } else if (count == 5)
                {
                    l = line.length();
                    obj.z = atof(line.substr(3,l-3).c_str());
                } else if (count == 6)
                {
                    l = line.length();
                    obj.type = line.substr(6,l-6);
                    ros::Duration(1).sleep();
                    object_pub_.publish(obj);
                    insertFileObject(obj);
                    count = -1;
                    ROS_INFO_STREAM("lol"<<obj);
                }
                count ++;
            }

            ifs.close();
         }


    }

    void insertSubObject(const vision_msgs::Object &new_object) {
        ROS_INFO("Inserting object");
        for (list<vision_msgs::Object>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {

            //If already inserted
            if(abs(new_object.x - it->x) < 0.03 && abs(new_object.y - it->y) < 0.03 && abs(new_object.z - it->z) < 0.03) {
                ROS_INFO("Already there, differences: %f, %f, %f", abs(new_object.x - it->x), abs(new_object.y - it->y), abs(new_object.z - it->z));
                return;
            }
        }
        ROS_INFO("Inserted new object");
        object_collector.push_back(new_object);
        echoObjects();
        object_pub_.publish(new_object);
        if(state == "WRITE")
            write2File(new_object);

    }





    void run()
    {
        if(state == "WRITE")
            remove("/home/ras/catkin_ws/src/wheatley/objects.txt");
        if(state == "READ")
            readFromFile();
        ros::spin();


    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_mapping_node");
    ros::NodeHandle nh;
    wheatley::MapNode map_node(nh);
    map_node.run();
}

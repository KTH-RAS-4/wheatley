#include <ros/ros.h>

#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/combine_grids.h>

#include <ros/wall_timer.h>

#include <tf/transform_listener.h>


namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;

using std::vector;
using std::string;
using std::list;
using std::abs;

class GridMergeNode {
private:

    ros::NodeHandle handle;
    ros::Subscriber sub_map;




    /****************************************
     * Params
     ****************************************/

    ros::NodeHandle nh_;


    /****************************************
     * Associated objects
     ****************************************/

    tf::TransformListener tf_;
    ros::Publisher pub_merged_grid;
    ros::WallTimer build_grid_timer_;
    boost::mutex mutex_;


    /****************************************
     * Stores
     ****************************************/

    nm::OccupancyGrid::ConstPtr current_map;
    nm::OccupancyGrid::ConstPtr merged_map;
    list<nm::OccupancyGrid::ConstPtr> map_collector;


public:
    GridMergeNode()
    {
        ROS_INFO("Running");

        pub_merged_grid = handle.advertise<nm::OccupancyGrid>("/merged_map", 100);

        sub_map = handle.subscribe("/map", 100, &GridMergeNode::mapCallback, this);


        build_grid_timer_ = handle.createWallTimer(ros::WallDuration(5), &GridMergeNode::echoGrid, this);

    }

    ~GridMergeNode()
    {
    }

    void echoGrid(const ros::WallTimerEvent& scan) {


    }

    void mapCallback(const nm::OccupancyGrid::ConstPtr &msg) {
        //current_map = msg;
        /*if(map_collector.size() > 1)
            map_collector.pop_back();

        map_collector.push_back(msg);*/


        vector<nm::OccupancyGrid::ConstPtr> input;
        /*input.push_back();
        for (list<nm::OccupancyGrid::ConstPtr>::iterator ci = map_collector.begin(); ci != map_collector.end(); ++ci) {
            input.push_back(*ci);
        }*/
        input.push_back(current_map);
        input.push_back(msg);

        nm::OccupancyGrid::ConstPtr merged_map = gu::combineGrids(input);
        current_map = merged_map;
        pub_merged_grid.publish(merged_map);
        ROS_INFO("Merged");
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_merge_node");
    GridMergeNode node;
    node.run();
}

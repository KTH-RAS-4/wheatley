#include <iostream>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vision_msgs/Object.h>
#include <vision_msgs/Objects.h>

float object_colors[] = {
    200, 180, 180,
    116.2592, 127.4758, 82.8304,
    173, 45, 56
};

std::string color_names[] = {
    "wall",
    "green",
    "red"
};

class ColorDetectorNode
{
    ros::NodeHandle handle;
    ros::Subscriber color_sub;

public:
    ColorDetectorNode()
    {
        color_sub = handle.subscribe("object_detection/objects", 1, &ColorDetectorNode::colorHandle, this);

        std::vector<float> v(3, 3);
        printVector(v);
    }

    ~ColorDetectorNode()
    {
    }

    void colorHandle(const vision_msgs::Objects& msg)
    {
        std::cout << "---------------------------" << std::endl;
        for (int i = 0; i < msg.objects.size(); i++)
        {
            int color = findClosestColor(msg.objects[i]);
            if(color != 0) {
                std::cout << "Object " << i << ": Color: " << color_names[color] << std::endl;
            }
        }
        if (msg.objects.size() == 0)
        {
            std::cout << "" << std::endl;
        }
        std::cout << "---------------------------" << std::endl;

    }

    int findClosestColor(vision_msgs::Object obj)
    {
        int n_colors = (sizeof(object_colors)/sizeof(*object_colors))/3;
        int color;
        float diff = 255*3;
        float newdiff;

        for (int i = 0; i < n_colors; i++)
        {
            newdiff = abs(obj.r-object_colors[i*3])+abs(obj.g-object_colors[i*3+1])+abs(obj.b-object_colors[i*3+2]);
            if (newdiff < diff)
            {
                diff = newdiff;
                color = i;
            }
        }

        return color;
    }

    void printVector(std::vector<float> v)
    {
        for (std::vector<float>::iterator it = v.begin(); it != v.end(); ++it)
        {
            std::cout << *it << ' ';
        }
        std::cout << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detector_node");
    ColorDetectorNode cdn;
    ros::spin();
    return 0;
}

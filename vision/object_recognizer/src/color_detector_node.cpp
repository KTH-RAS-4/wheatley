#include <iostream>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <ras_arudino_msgs/object.h>
#include <ras_arudino_msgs/objects.h>

float object_colors[] = {
    116.2592, 127.4758, 82.8304,
    173, 45, 56
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

    void colorHandle(const ras_arudino_msgs::Objects& msg)
    {
        for (int i = 0; i < sizeof(msg.objects)/sizeof(*msg.objects); i++)
        {
            int color = findClosestColor(msg.objects[i]);
            std::cout << color << std::endl;
        }
    }

    int findClosestColor(ras_arudino_msgs::Object obj)
    {
        int n_colors = sizeof(object_colors)/sizeof(*object_colors);
        int color;
        float diff = 255*3;
        float newdiff;

        for (int i = 0; i < n_colors; i++)
        {
            newdiff = Math.abs(obj.r-object_colors[i*3])+Math.abs(obj.g-object_colors[i*3+1])+Math.abs(obj.b-object_colors[i*3+2]);
            if (newdiff < diff)
            {
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
    //ros::spin();
    return 0;
}

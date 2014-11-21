#include <iostream>
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vision_msgs/Object.h>
#include <vision_msgs/Objects.h>
#include <map>

using namespace std;

float object_colors[] = {
    180, 150, 130,
    220, 220, 220,
    116.2592, 127.4758, 82.8304,
    200, 90, 100,
    220, 120, 120,
    250, 150, 140
};

int relevant_iterations = 10;

std::string color_names[] = {
    "wall",
    "wall",
    "green cube",
    "red cube",
    "red ball",
    "patrick"
};

class StoredObject
{
    vision_msgs::Object object;
    map<int, bool> occ;

public:
    StoredObject(vision_msgs::Object object, int iteration) {
        this->object = object;
        occ[iteration] = true;
    }
    ~StoredObject() {

    }

    bool compare(vision_msgs::Object other) {
        if(abs(other.x - object.x) < 0.01 && abs(other.y - object.y) < 0.01 && abs(other.z - object.z) < 0.01) {
            return true;
        } else {
            return false;
        }
    }
    void update(vision_msgs::Object obj, int iteration) {
        occ[iteration] = true;
        this->object = obj;

        //Delete old detections
        if(occ.find(iteration-relevant_iterations) != occ.end())
            occ.erase(iteration-relevant_iterations);
    }

    int occurrencyCount(int iteration) {
        int counter = 0;
        for(int i = (iteration-(relevant_iterations-1)); i<=iteration; i++) {
            if(occ.find(i) != occ.end()) {
                counter++;
            }
        }
        return counter;
    }
    vision_msgs::Object getObject() {
        return this->object;
    }
};

class ColorDetectorNode
{
    ros::NodeHandle handle;
    ros::Subscriber object_sub;
    list<StoredObject> object_collector;
    int iteration;

public:
    ColorDetectorNode()
    {
        iteration = 0;
        object_sub = handle.subscribe("object_detection/objects", 1, &ColorDetectorNode::objectHandle, this);
    }

    ~ColorDetectorNode()
    {
    }

    void objectHandle(const vision_msgs::Objects& msg)
    {
        int new_objects = 0;
        //Insert objects into collection
        for (int i = 0; i < msg.objects.size(); i++)
        {
            if(storeObject(msg.objects[i]))
                new_objects++;
        }
        cout << "Iteration " << iteration << " - " << new_objects << "/" << msg.objects.size() << " new objects" << endl;

        //Output objects with multiple occurrencies

        for (list<StoredObject>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {
            int obj_iter = 0;
            if((obj_iter = it->occurrencyCount(iteration)) > (relevant_iterations / 2)) {
                vision_msgs::Object obj = it->getObject();
                int color = findClosestColor(obj);
                if (color_names[color ]!= "wall") {
                    cout << "Found " << color_names[color] << " (" << obj.r << ", " << obj.g << ", " << obj.b << ")" << endl;
                } else {
                    cout << "Found wall (" << obj.r << ", " << obj.g << ", " << obj.b << ")" << endl;
                }
            } else if (obj_iter == 0) {
                it = object_collector.erase(it);
            }
        }

        iteration++;
    }

    bool storeObject(vision_msgs::Object obj)
    {
        for (list<StoredObject>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {
            if(it->compare(obj)) {
                it->update(obj, iteration);
                return false;
            }
        }
        object_collector.push_back(StoredObject(obj, iteration));
        return true;
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detector_node");
    ColorDetectorNode cdn;
    ros::spin();
    return 0;
}

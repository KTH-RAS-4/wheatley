#include <iostream>
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vision_msgs/Object.h>
#include <vision_msgs/Objects.h>
#include <map>
#include <ras_msgs/RAS_Evidence.h>
//#include <std_msgs/String.h>
#include <sound_play/SoundRequest.h>

using namespace std;

float object_color_array[] = {
    180, 150, 130,
    200, 90, 100,
    116.2592, 127.4758, 82.8304,
    220, 120, 120,
    250, 150, 140,
    170, 120, 90,
    140, 170, 10
};

map<int, string > object_color_map;

int relevant_iterations = 10;
sensor_msgs::Image currentImage;

string object_types[] = {
    "Wall",
    "Red Cube",
    "Green Cube",
    "Red Ball",
    "Patric",
    "Blue Cube",
    "Yellow Cube",
    "Yellow Ball",
    "Green Cylinder",
    "Blue Triangle",
    "Purple Cross"
};

ros::Publisher pub_evidence;
ros::Publisher pub_speaker;

class StoredObject
{
    vision_msgs::Object object;
    map<int, bool> occ;
    bool identified;
    string type;

public:
    StoredObject(vision_msgs::Object object, int iteration) {
        this->object = object;
        occ[iteration] = true;
        identified = false;
        type = "nothing";

        //Init color map
        object_color_map[0] = "Wall";
        object_color_map[1] = "Red Cube";
        object_color_map[2] = "Green Cube";
        object_color_map[3] = "Red Ball";
        object_color_map[4] = "Patrick";
        object_color_map[5] = "Wall";
        object_color_map[6] = "Green Cylinder";

    }
    ~StoredObject() {

    }

    bool compare(vision_msgs::Object other) {
        if(abs(other.x - object.x) < 0.03 && abs(other.y - object.y) < 0.03 && abs(other.z - object.z) < 0.03) {
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

    void publish() {
        if(!identified) {
            identified = true;
            cout << "Found " << type << " (" << object.x << ", " << object.y << ", " << object.z << ")" << " - Color (" << object.r << ", " << object.g << ", " << object.b << ")" << endl;

            ras_msgs::RAS_Evidence evidence;
            evidence.group_number = 4;
            evidence.stamp = ros::Time::now();
            evidence.image_evidence = currentImage;
            evidence.object_id = this->type;
            pub_evidence.publish(evidence);

            sound_play::SoundRequest message;
            message.sound = -3;
            message.command = 1;
            message.arg = "I see a " + type;
            pub_speaker.publish(message);
        } else {
            cout << "Found " << type << " again" << endl;
        }

    }

    vision_msgs::Object getObject() {
        return this->object;
    }

    string getType(int color) {
        if(this->type == "nothing") {
            this->type = object_color_map.find(color)->second;
        }
        return this->type;
    }
    string getType() {
        return this->type;
    }
    bool isIdentified() {
        return this->identified;
    }
};

class ColorDetectorNode
{
    ros::NodeHandle handle;
    ros::Subscriber sub_object;
    ros::Subscriber sub_image;
    list<StoredObject> object_collector;
    int iteration;

public:
    ColorDetectorNode()
    {
        iteration = 0;
        pub_evidence = handle.advertise<ras_msgs::RAS_Evidence> ("/evidence", 1);
        //pub_speaker = handle.advertise<std_msgs::String> ("/espeak/string", 1);
        pub_speaker = handle.advertise<sound_play::SoundRequest>("robotsound", 1);
        sub_image = handle.subscribe("camera/rgb/image_raw", 1, &ColorDetectorNode::storeImage, this);
        sub_object = handle.subscribe("object_detection/objects", 1, &ColorDetectorNode::objectHandle, this);
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

                string type = it->getType(findClosestColor(obj));

                if (type != "Wall") {
                    it->publish();
                } else {
                    cout << "Found wall (" << obj.r << ", " << obj.g << ", " << obj.b << ")" << endl;
                }
            } else if (obj_iter == 0 && !it->isIdentified()) {
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

    void storeImage(const sensor_msgs::Image& msg) {
        currentImage = msg;
    }



    int findClosestColor(vision_msgs::Object obj)
    {
        int n_colors = (sizeof(object_color_array)/sizeof(*object_color_array))/3;
        int color;
        float diff = 255*3;
        float newdiff;

        for (int i = 0; i < n_colors; i++)
        {
            newdiff = abs(obj.r-object_color_array[i*3])+abs(obj.g-object_color_array[i*3+1])+abs(obj.b-object_color_array[i*3+2]);
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

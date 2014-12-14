#include <iostream>
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vision_msgs/Object.h>
#include <vision_msgs/Objects.h>
#include <map>
#include <ras_msgs/RAS_Evidence.h>
#include <std_msgs/String.h>
//#include <sound_play/SoundRequest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

// objects
int WALL = 0x0;
int ORANGE = 0x1;
int YELLOW = 0x2;
int PURPLE = 0x3;
int BLUE = 0x4;
int GREEN = 0x5;
int RED = 0x6;
int LIGHT_GREEN = 0x7;

int objects_list[] = {
    WALL,
    ORANGE,
    YELLOW,
    PURPLE,
    BLUE,
    GREEN,
    RED,
    LIGHT_GREEN
};

// types
int CIRCLE = 0x10;
int COMPLEX = 0x30;

// hue min max values
float object_color_hue[] = {
    -20, 180, // wall
    0, 10, // orange
    10, 25, // yellow
    125, 160, // purple
    70, 125, // blue
    25, 70, // green
    -20, -1, // red
    25, 70 //light green
};

// saturation min max values
float object_color_saturation[] = {
    0, 80,      // wall
    80, 255,    // not wall
};

float object_color_value[] = {
    0, 255,
    0, 0,
    160, 255,   //yellow
    0, 0,
    0, 0,
    0, 179,     //green
    0, 0,
    180, 255    //light green
};

map<string, bool> object_found;
map<int, string > object_color_map;

int relevant_iterations = 10;
sensor_msgs::Image currentImage;

ros::Publisher pub_evidence;
ros::Publisher pub_speaker;
ros::Publisher pub_detected_object;
ros::Publisher pub_colored_object;

class StoredObject
{
    vision_msgs::Object object;
    map<int, bool> occ;
    bool identified;
    bool sent_to_shape;
    string type;
    string color;

public:
    StoredObject(vision_msgs::Object object, int iteration) {
        this->object = object;
        occ[iteration] = true;
        identified = false;
        sent_to_shape = false;
        type = "nothing";
        color = "";

        //Init color map
        object_color_map[WALL] = "Wall";
        object_color_map[ORANGE] = "Orange";
        object_color_map[YELLOW] = "Yellow";
        object_color_map[GREEN] = "Green";
        object_color_map[BLUE] = "Blue";
        object_color_map[PURPLE] = "Purple";
        object_color_map[RED] = "Red";
        object_color_map[LIGHT_GREEN] = "Light Green";

    }
    ~StoredObject() {

    }

    bool compare(vision_msgs::Object other) {
        if(abs(other.x - object.x) < 0.05 && abs(other.y - object.y) < 0.05 && abs(other.z - object.z) < 0.05) {
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

    void identify() {
        if(!sent_to_shape && !identified) {
            if(type == "Light Green") {
                type = object.type = "Green Cylinder";
                publish();
                return;
            } else if(type == "Green") {
                type = object.type = "Green Cube";
                publish();
                return;
            } else if(type == "Purple") {
                type = object.type = "Purple Cross";
                publish();
                return;
            } else if(type == "Orange") {
                type = object.type = "Patric";
                publish();
                return;
            } else {
                pub_colored_object.publish(object);
                sent_to_shape = true;
            }
        }

    }

    void publish() {
        if(!identified && !object_found[this->type]) {
            object_found[this->type] = true;
            ROS_INFO("Publishing");
            pub_detected_object.publish(this->object);
            cout << "Found " << object.type << endl;
            identified = true;
            /*cout << "Found " << object.type << //" (" << object.x << ", " << object.y << ", " << object.z << ")" <<
                    " - Color (" << object.r << ", " << object.g << ", " << object.b << "), GrayColor: " << (.11*object.r + .59*object.g + 0.3*object.b) << endl;
            */
            ras_msgs::RAS_Evidence evidence;
            evidence.group_number = 4;
            evidence.stamp = ros::Time::now();
            evidence.image_evidence = currentImage;
            evidence.object_id = this->type;
            pub_evidence.publish(evidence);

            std_msgs::String message;
            message.data = "I see a " + type;
            pub_speaker.publish(message);

        } else {
            cout << "Found " << type << " again" << endl;
        }
    }

    vision_msgs::Object getObject() {
        return this->object;
    }

    string getType(int type) {
        if(this->type == "nothing") {
            if (type != -1)
            {
                int object = type & 0xF;
                int shape = type & 0xF0;
                this->type = object_color_map.find(object)->second;
            }
            else
            {
                this->type = "object";
            }
            object.type = this->type;
        }
        color = this->type;
        return this->type;
    }
    string getType() {
        return this->type;
    }
    void setType(string type) {
        this->type = object.type = type;
    }

    bool isIdentified() {
        return this->identified;
    }
};

class ObjectCollectorNode
{
    ros::NodeHandle handle;
    ros::Subscriber sub_object;
    ros::Subscriber sub_image;
    ros::Subscriber sub_identified_object;
    list<StoredObject> object_collector;
    int iteration;

public:
    ObjectCollectorNode()
    {
        iteration = 0;
        pub_evidence = handle.advertise<ras_msgs::RAS_Evidence> ("/evidence", 1);
        pub_speaker = handle.advertise<std_msgs::String> ("/espeak/string", 100);
        pub_detected_object = handle.advertise<vision_msgs::Object>("/object_recognition/objects", 100);
        pub_colored_object = handle.advertise<vision_msgs::Object>("/object_detection/detected_objects", 100);
        sub_image = handle.subscribe("camera/rgb/image_raw", 1, &ObjectCollectorNode::storeImage, this);
        sub_object = handle.subscribe("object_detection/objects", 1, &ObjectCollectorNode::objectHandle, this);
        sub_identified_object = handle.subscribe("object_recognition/detected_objects", 1, &ObjectCollectorNode::shapeDetected, this);

        object_found["Red Cube"] = false;
        object_found["Red Ball"] = false;
        object_found["Yellow Cube"] = false;
        object_found["Yellow Ball"] = false;
        object_found["Green Cube"] = false;
        object_found["Green Cylinder"] = false;
        object_found["Patric"] = false;
        object_found["Purple Cross"] = false;
        object_found["Blue Triangle"] = false;
        object_found["Blue Cube"] = false;
    }

    ~ObjectCollectorNode()
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
        cout << " ----------------------------------- " << iteration << " ----------------------------------- "<< endl;

        //Output objects with multiple occurrencies
        for (list<StoredObject>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {
            int obj_iter = 0;
            if((obj_iter = it->occurrencyCount(iteration)) > (relevant_iterations / 2)) {
                vision_msgs::Object obj = it->getObject();

                string type = it->getType(findObjectType(obj));

                if (type != "Wall") {
                    it->identify();
                }
            } else if (obj_iter == 0 && !it->isIdentified()) {
                it = object_collector.erase(it);
            }
        }

        iteration++;
    }
    void shapeDetected(const vision_msgs::Object &msg) {
        for (list<StoredObject>::iterator it = object_collector.begin() ; it != object_collector.end(); ++it) {
            if(it->compare(msg)) {
                it->setType(msg.type);
                it->publish();
                return;
            }
        }
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

    int findObjectType(vision_msgs::Object obj)
    {
        int type = -1;

        // extract hsv
        Mat bgr = Mat(1, 1, CV_8UC3, Scalar(obj.b, obj.g, obj.r));
        Mat hsv;
        cvtColor(bgr, hsv, CV_BGR2HSV);
        Vec3b pix = hsv.at<Vec3b>(0,0);
        int h = pix[0]; // [0, 180]
        int s = pix[1]; // [0, 255]
        int v = pix[2]; // [0, 255]
        int _h;

        int object;
        int shape;

        for (int i = 0; i <= (sizeof(objects_list)/sizeof(*objects_list)); i++)
        {
            // object / color detection
            object = objects_list[i];
            _h = h > object_color_hue[object*2+1] ? h-180 : h; // wrap the value
            if (_h >= object_color_hue[object*2] && _h <= object_color_hue[object*2+1])
            {
                if (object == WALL)
                { 
                    if (s >= object_color_saturation[object*2] && s <= object_color_saturation[object*2+1])
                    {
                        type = object;
                    }
                } else if(object == YELLOW || object == LIGHT_GREEN || object == GREEN) {
                    if (v >= object_color_value[object*2] && v <= object_color_value[object*2+1])
                    {
                        type = object;
                    } else {
                        type = WALL;
                    }
                }
                else
                {
                    type = object;
                }
            }
        }
        ROS_INFO_STREAM("Found object " << object_color_map[type] << ", (" << h << ", " << s << ", " << v << ")");

        return type;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_collector_node");
    ObjectCollectorNode cdn;
    ros::Duration(5).sleep();
    ros::spin();
    return 0;
}

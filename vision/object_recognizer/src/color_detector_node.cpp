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

int objects_list[] = {
    WALL,
    ORANGE,
    YELLOW,
    PURPLE,
    BLUE,
    GREEN,
    RED
};

// types
int CIRCLE = 0x10;
int COMPLEX = 0x30;

float object_color_array[] = {
    220, 220, 220,
    200, 90, 100,   //Red Cube
    88, 120, 68,    //Green Cube
    220, 120, 120,  //Red Sphere
    250, 150, 140,  //Patrick
    //170, 120, 90,
    140, 170, 10,   //Green Cylinder
    //220, 220, 220,
    //111, 105, 102,
    220, 175, 130,  //Yellow Cube
    125, 801, 155,  //Purple Cross
    55, 85, 50,     //Green Cube
    100, 120, 160   //Blue Cube
};

// hue min max values
float object_color_hue[] = {
    -10, 180, // wall
    5, 10, // orange
    15, 25, // yellow
    125, 160, // purple
    50, 125, // blue
    25, 50, // green
    -8, 5 // red
};

// saturation min max values
float object_color_saturation[] = {
    0, 80, // wall
    80, 255 // not wall
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
ros::Publisher pub_object;

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
        /*object_color_map[0] = "Wall";
        object_color_map[1] = "Red Cube";
        object_color_map[2] = "Green Cube";
        object_color_map[3] = "Red Ball";
        object_color_map[4] = "Patrick";
        //object_color_map[5] = "Wall";
        object_color_map[5] = "Green Cylinder";
        //object_color_map[7] = "Wall";
        //object_color_map[8] = "Wall";
        object_color_map[6] = "Yellow Cube";
        object_color_map[7] = "Purple Cross";
        object_color_map[8] = "Green Cube";
        object_color_map[9] = "Blue Cube";*/

        object_color_map[WALL] = "Wall";
        object_color_map[ORANGE] = "Patrick";
        object_color_map[YELLOW] = "Yellow";
        object_color_map[GREEN] = "Green";
        object_color_map[BLUE] = "Blue";
        object_color_map[PURPLE] = "Purple";
        object_color_map[RED] = "Red";
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
            cout << "Found " << type << //" (" << object.x << ", " << object.y << ", " << object.z << ")" <<
                    " - Color (" << object.r << ", " << object.g << ", " << object.b << "), GrayColor: " << (.11*object.r + .59*object.g + 0.3*object.b) << endl;

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

            pub_object.publish(this->object);

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
                this->type = "Undefined";
            }
            this->object.type = this->type;
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
        pub_object = handle.advertise<vision_msgs::Object>("/object_recognition/objects", 100);
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

                //string type = it->getType(findClosestColor(obj));
                string type = it->getType(findObjectType(obj));

                if (type != "Wall") {
                    it->publish();
                } else {
                    //cout << "Found wall (" << obj.r << ", " << obj.g << ", " << obj.b << "), GrayColor: " << (.11*obj.r + .59*obj.g + 0.3*obj.b) << endl;
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

        int threshold = 15;
        float average = (obj.r + obj.g + obj.b) / 3;
        if(abs(obj.r-average) < threshold && abs(obj.g-average) < threshold && abs(obj.b-average) < threshold)  {
            return 0;
        }


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
                }
                else
                {
                    type = object;
                }
            }

            // TODO: shape detection
            // type = type | shape;
        }
        ROS_INFO_STREAM("Found object " << type << ", (" << h << ", " << s << ", " << v << ")");

        return type;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detector_node");
    ColorDetectorNode cdn;
    ros::spin();
    return 0;
}

#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <dirent.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Object.h>
#include <vision_msgs/Objects.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

static const string WINDOW_1 = "Image mask";
static const string WINDOW_2 = "Image";

static const string IMAGE_DIR = "catkinws/src/wheatley/vision/object_recognizer/img/";

bool visual_debugging = false;
bool verbose = true;

double approx_epsilon = 5;

// objects
static const int WALL = 0x0;
//static const int FLOOR = 0x1;
static const int RED = 0x1;
static const int ORANGE = 0x2;
static const int YELLOW = 0x3;
static const int GREEN = 0x4;
static const int BLUE = 0x5;
static const int PURPLE = 0x6;

// shapes
static const int TRIANGLE = 0x10;

string color_names[] = {
    "Wall",
//    "Floor",
    "Red",
    "Orange",
    "Yellow",
    "Green",
    "Blue",
    "Purple"
};

string simple_shapes[] = {
    "", // wall
    "Cube", // red
    "", // orange
    "Cube", // yellow
    "Cube", // green
    "Cube", // blue
    "" // purple
};

string complex_shapes[] = {
    "", // wall
    "Ball", // red
    "Star", // orange
    "Ball", // yellow
    "Cylinder", // green
    "Triangle", // blue
    "Cross" // purple
};

// hue min max values
float object_color_hue[] = {
    -10, 180, // wall
//  0, 180, // floor
    -8, 0, // red
    1, 10, // orange
    15, 30, // yellow
    30, 70, // green
    70, 125, // blue
    125, 160 // purple
};

// saturation min max values
float object_color_saturation[] = {
    0, 120, // wall
//    0, 255, // floor
    90, 255, // red
    120, 255, // orange
    140, 255, // yellow
    60, 255, // green
    60, 255, // blue
    80, 255 // purple
};

// value min max values
float object_color_value[] = {
    50, 255, // wall
//    0, 255, // floor
    70, 255, // red
    127, 255, // orange
    127, 255, // yellow
    60, 255, // green
    50, 255, // blue
    80, 255 // purple
};

/*
// hue min max values
float object_color_hue[] = {
    0, 180, // wall
    0, 180, // floor
    160, 180, // red
    0, 10, // orange
    17, 30, // yellow
    30, 80, // green
    80, 120, // blue
    120, 160 // purple
};

// saturation min max values
float object_color_saturation[] = {
    0, 80, // wall
//    80, 255, // floor
    0, 255, // red
    0, 255, // orange
    0, 255, // yellow
    0, 255, // green
    0, 255, // blue
    0, 255 // purple
};

// value min max values
float object_color_value[] = {
    0, 255, // wall
//    0, 255, // floor
    0, 255, // red
    0, 255, // orange
    0, 255, // yellow
    0, 255, // green
    0, 255, // blue
    0, 255 // purple
};

*/

int mask_color_list[] = {
    WALL
};

int color_list[] = {
    WALL,
    ORANGE,
    YELLOW,
    PURPLE,
    RED,
    GREEN,
    BLUE
};

int detect_iterations = 10;
int detect_id = 0;

struct detected_object {
    int color;
    int iterations;
	int candidates[2];
    map<int, int> vertices;
	vision_msgs::Object object;
};

class ImageObjectDetectionNode
{
    ros::NodeHandle handle;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    ros::Subscriber object_sub;
    ros::Publisher object_pub;

    Mat image;
    Mat result;
    vector<Mat> channels;

    Mat erosion_element;
    Mat dilation_element;

    map<int, detected_object> detected_objects;

    vector<Mat> triangle_imgs;

public:
    ImageObjectDetectionNode()
    : it(handle)
    {
        object_pub = handle.advertise<vision_msgs::Object>("/object_recognition/detected_objects", 100);
        image_sub = it.subscribe("/camera/rgb/image_raw", 1, &ImageObjectDetectionNode::imageHandle, this);
        object_sub = handle.subscribe("object_detection/detected_objects", 1, &ImageObjectDetectionNode::objectHandle, this);

        if (visual_debugging)
        {
            namedWindow(WINDOW_1);
            namedWindow(WINDOW_2);
        }

        int erosion_size = 3;
        erosion_element = getStructuringElement(MORPH_ELLIPSE, Size(2*erosion_size+1, 2*erosion_size+1), Point(erosion_size, erosion_size));
        int dilation_size = 5;
        dilation_element = getStructuringElement(MORPH_ELLIPSE, Size(2*dilation_size+1, 2*dilation_size+1), Point(dilation_size, dilation_size));

        DIR *dir;
        struct dirent *ent;
        Mat img;
        string dir_str;
        dir_str = IMAGE_DIR + "triangle/";
        if ((dir = opendir (dir_str.c_str())) != NULL) {
            while ((ent = readdir (dir)) != NULL) {
                if (strcmp(ent->d_name, ".") && strcmp(ent->d_name, ".."))
                {
                    img = imread(dir_str + ent->d_name, CV_LOAD_IMAGE_GRAYSCALE);
                    equalizeHist(img, img);
                    //cvtColor(img, img, CV_GRAY2BGR);
                    GaussianBlur(img, img, Size(3, 3), 3, 0, BORDER_DEFAULT);
                    //cvtColor(img, img, CV_BGR2GRAY);
                    img.convertTo(img, CV_8U);
                    triangle_imgs.push_back(img);
                }
            }
            closedir (dir);
        }
    }

    ~ImageObjectDetectionNode()
    {
        if (visual_debugging)
        {
            destroyWindow(WINDOW_1);
            destroyWindow(WINDOW_2);
        }
    }

private:
    void imageHandle(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr frame;

        try
        {
            frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            frame->image.copyTo(image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (detected_objects.size() > 0)
        {
            // normalize the colors
            //cvtColor(image, image, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format
            //split(image, channels); //split the image into channels
            //equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)
            //merge(channels, image); //merge 3 channels including the modified 1st channel into one image
            //cvtColor(image, image, CV_YCrCb2BGR); //change the color image from YCrCb to BGR format (to display image properly)

            // extract wall sample color
            /*Mat bin, bin_sat, bin_val;
            cvtColor(image, image, CV_BGR2HSV);
            vector<Mat> channels;
            split(image, channels);
            threshold(channels[1], bin_sat, 80, 255, 0);
            bitwise_not(bin_sat, bin_sat);
            threshold(channels[2], bin_val, 200, 255, 0);
            cvtColor(image, image, CV_HSV2BGR);
            bitwise_and(bin_sat, bin_val, bin, noArray());
            Scalar average = mean(image, bin);

            // color correction based on sample color
            split(image, channels);
            double min, max, wmax;
            wmax = 230;
            //cout << "-----" << endl;
            minMaxLoc(channels[0], &min, &max);
            normalize(channels[0], channels[0], min, max + wmax - average[0], NORM_MINMAX, -1, noArray());
            //cout << max + wmax - average[2] << " (" << wmax - average[0] << ")" << endl;
            minMaxLoc(channels[1], &min, &max);
            normalize(channels[1], channels[1], min, max + wmax - average[1], NORM_MINMAX, -1, noArray());
            //cout << max + wmax - average[1] << " (" << wmax - average[1] << ")" << endl;
            minMaxLoc(channels[2], &min, &max);
            normalize(channels[2], channels[2], min, max + wmax - average[2], NORM_MINMAX, -1, noArray());
            //cout << max + wmax - average[2] << " (" << wmax - average[2] << ")" << endl;
            merge(channels, image);*/
            
            // apply gaussian filter
            GaussianBlur(image, image, Size(9, 9), 3, 0, BORDER_DEFAULT);

            // erosion
            //erode(image, image, erosion_element);
            // dilation
            //dilate(image, image, dilation_element);

            // loop over the detected objects
            for (map<int, detected_object>::iterator it = detected_objects.begin(); it != detected_objects.end(); )
            {
                detected_object obj = it->second;
                int color = obj.color;

				cout << "checking " << obj.object.type << " iteration " << obj.iterations << endl;

                // detect objects
                Mat marker;
                Mat mask = maskColor(image, color);
                erode(mask, marker, erosion_element);
                mask = maskReconstruction(marker, mask);
                result = Mat::zeros(image.rows, image.cols, CV_8UC3);
                Vec3b bgr = Vec3b(255,255,255);
                Vec3b* _result;
                float* _mask;
                for (int i = 0; i < image.rows; i++)
                {
                    _result = result.ptr<Vec3b>(i);
                    _mask = mask.ptr<float>(i);
                    for (int j = 0; j < image.cols; j++)
                    {
                        if (_mask[j] > 0)
                        {
                            _result[j] = bgr;
                        }
                    }
                }

                // edge detection
                Mat edges;
                Canny(result, edges, 20, 40, 3, false);
                /// Find contours
                vector<vector<Point> > contours;
                vector<Vec4i> hierarchy;
                findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
                // find polygons
                vector<vector<Point> > polygons;
                vector<Point> polygon;
                vector<Rect> boundingBoxes;
                Rect bounds;
                int boundingThreshold = 10;
                for (int i = 0; i < contours.size(); i++)
                {
                    approxPolyDP(contours[i], polygon, approx_epsilon, true);
                    bounds = boundingRect(Mat(polygon));
                    bool match = false;
                    for (int j = 0; j < boundingBoxes.size() && j < polygons.size(); )
                    {
                        // if the new bounds are enclosing
                        if (bounds.x-boundingThreshold <= boundingBoxes[j].x &&
                            bounds.y-boundingThreshold <= boundingBoxes[j].y &&
                            bounds.width+boundingThreshold >= boundingBoxes[j].width &&
                            bounds.height+boundingThreshold >= boundingBoxes[j].height)
                        {
                            //rectangle(result, boundingBoxes[j], Scalar(0,0,255), 1, 8, 0);
                            polygons.erase(polygons.begin()+j);
                            boundingBoxes.erase(boundingBoxes.begin()+j);
                        }
                        // if the new bounds is enclosed
                        else if (bounds.x >= boundingBoxes[j].x &&
                            bounds.y >= boundingBoxes[j].y &&
                            bounds.width <= boundingBoxes[j].width &&
                            bounds.height <= boundingBoxes[j].height)
                        {
                            match = true;
                            break;
                        } else {
                            j++;
                        }
                    }
                    if (!match)
                    {
                        polygons.push_back(polygon);
                        boundingBoxes.push_back(bounds);
                    }
                }
                polylines(result, polygons, true, Scalar(00,255), 1, 8, 0);
                int foundidx = -1, foundsize = 0;
                for (int i = 0; i < boundingBoxes.size() && i < polygons.size(); i++)
                {
                    if (boundingBoxes[i].width*boundingBoxes[i].height > foundsize)
                    {
                        foundidx = i;
                        foundsize = boundingBoxes[i].width*boundingBoxes[i].height;
                    }
                    rectangle(result, boundingBoxes[i], Scalar(255,0,255), 1, 8, 0);
                }

                if (visual_debugging)
                {
                    imshow(WINDOW_2, result);
                }

                /*ROS_INFO_STREAM("Polygons size = " << polygons.size());
                if(polygons.size() == 0) {
                    obj.iterations++;
                    it->second = obj;
                    it++;
                    return;
                }*/

                int vertices;

				/*
                vertices = polygons[foundidx].size();
                if (obj.vertices.find(vertices) == obj.vertices.end())
                {
                    obj.vertices[vertices] = 0;
                }
                obj.vertices[vertices]++;*/
                if (foundidx != -1)
                {
                    if (color == BLUE)
                    {
                        // extract the masked part of the image
                        int isShape = checkObjectShape(image(boundingBoxes[foundidx]), TRIANGLE);
                        if (isShape)
                        {
                            // found blue triangle (complex shape)
                            obj.candidates[1]++;
                        }
                        else
                        {
                            // found blue cube (simple shape)
                            obj.candidates[0]++;
                        }
                    }
                    else
                    {
                        if (polygons[foundidx].size() <= 6)
        				{
        					// found simple shape
        					obj.candidates[0]++;
        				}
        				else
        				{
        					// found complex shape
        					obj.candidates[1]++;
        				}
                    }
                }
                obj.iterations++;

                if (verbose && polygons.size() != 0)
				{
                    cout << "found " << color_names[color] << " object with " << polygons[foundidx].size()<< " vertices (" << obj.candidates[0] << " : " << obj.candidates[1] << ")" << endl;
				}

                if (obj.iterations >= detect_iterations)
                {
					int highest_count = 0;
                    /*
                    for (map<int, int>::iterator vit = obj.vertices.begin(); vit != obj.vertices.end(); vit++)
                    {
                        if (vit->second > highest_count)
                        {
                            vertices = vit->first;
                            highest_count = vit->second;
                        }
                    }*/

                    if (obj.candidates[0] > obj.candidates[1])
                    {
                        // found simple shape
						highest_count = obj.candidates[0];
                        obj.object.type += " " + simple_shapes[color];
                    }
                    else
                    {
                        // found complex shape
						highest_count = obj.candidates[1];
                        obj.object.type += " " + complex_shapes[color];
                    }

                    cout << "published " << obj.object.type << " (" << highest_count << " candidates)" << endl;

                    object_pub.publish(obj.object);

                    detected_objects.erase(it++);
                    //it--;
                }
				else
				{
					it->second = obj;
                    it++;
				}

            }
        }

        if (visual_debugging)
        {
            imshow(WINDOW_1, image);
        }

        waitKey(10);
    }

    void objectHandle(const vision_msgs::Object& msg)
    {
        int color = -1;

        if (msg.type == "Red")
            color = RED;
        else if (msg.type == "Orange")
            color = ORANGE;
        else if (msg.type == "Yellow")
            color = YELLOW;
        else if (msg.type == "Green")
            color = GREEN;
        else if (msg.type == "Blue")
            color = BLUE;
        else if (msg.type == "Purple")
            color = PURPLE;

        if (verbose) {
            cout << "--- got detect request (" << msg.type << ")" << endl;
        }

        if (color < 0)
        {
            if (verbose) {
                cout << "color not detected [" << msg.r << " " << msg.g << " " << msg.b << "]" << endl;
            }
            return;
        }

        // add the object to be recognized
        detected_object obj;
		obj.color = color;
		obj.iterations = 0;
		obj.candidates[0] = 0;
		obj.candidates[1] = 0;
		obj.object = msg;
        detected_objects[detect_id] = obj;
        detect_id++;
    }

    /**
     * Returns -1 if no match was found.
     */
    int getColor(Vec3b src)
    {
        // extract hsv
        Mat bgr = Mat(1, 1, CV_8UC3, Scalar(src[0], src[1], src[2]));
        Mat hsv;
        cvtColor(bgr, hsv, CV_BGR2HSV);
        Vec3b pix = hsv.at<Vec3b>(0,0);
        int h = pix[0]; // [0, 180]
        int s = pix[1]; // [0, 255]
        int v = pix[2]; // [0, 255]
        int _h;

        int c;
        int color = -1;
        for (int i = 0; i < (sizeof(color_list)/sizeof(*color_list)); i++)
        {
            c = color_list[i];
            _h = h > object_color_hue[c*2+1] ? h-180 : h; // wrap the value
            if (_h >= object_color_hue[c*2] && _h <= object_color_hue[c*2+1])
            { 
                if (s >= object_color_saturation[c*2] && s <= object_color_saturation[c*2+1])
                {
                    if (v >= object_color_value[c*2] && v <= object_color_value[c*2+1])
                    {
                        color = c;
                        break;
                    }
                }
            }
        }

        return color;
    }

    Mat maskColor(const Mat src, int color)
    {
        Mat Tmp;

        cvtColor(src, Tmp, CV_BGR2HSV);

        Mat mask = Mat::zeros(src.rows, src.cols, CV_32FC1);

        int type;
        int h, _h, s, v;
        Vec3b* _Tmp;
        float* _mask;

        for (int i = 0; i < src.rows; i++)
        {
            _Tmp = Tmp.ptr<Vec3b>(i);
            _mask = mask.ptr<float>(i);
            for (int j = 0; j < src.cols; j++)
            {
                type = 0;
                h = _Tmp[j][0];
                s = _Tmp[j][1];
                v = _Tmp[j][2];
                _h = h > object_color_hue[color*2+1] ? h-180 : h; // wrap the value
                if (_h >= object_color_hue[color*2] && _h <= object_color_hue[color*2+1])
                { 
                    if (s >= object_color_saturation[color*2] && s <= object_color_saturation[color*2+1])
                    {
                        if (v >= object_color_value[color*2] && v <= object_color_value[color*2+1])
                        {
                            type = 1;
                        }
                    }
                }
                _mask[j] = type;
            }
        }

        return mask;
    }

    Mat maskReconstruction(Mat marker, Mat mask)
    {
        int rows = mask.rows;
        int cols = mask.cols;
        Mat mat = Mat::zeros(rows, cols, CV_32FC1);

        float* _marker = marker.ptr<float>(0);
        float* _mask = mask.ptr<float>(0);
        float* _mat = mat.ptr<float>(0);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (_marker[i*cols+j] == 1 && _mask[i*cols+j] == 1 && _mat[i*cols+j] == 0)
                {
                    _mat[i*cols+j] = 1;
                    //maskReconstructionHelper(_mask, _mat, rows, cols, i, j);
                }
            }
        }

        return mat;
    }

    void maskReconstructionHelper(float* src, float* dst, int rows, int cols, int r, int c)
    {
        for (int i = -1; i <= 1; i++)
        {
            if (r+i >= 0 && r+i < rows)
            {
                for (int j = -1; j <= 1; j++)
                {
                    if (c+j >= 0 && c+j < cols)
                    {
                        if (src[(r+i)*cols+c+j] == 1 && dst[(r+i)*cols+c+j] == 0)
                        {
                            dst[(r+i)*cols+c+j] = 1;
                            maskReconstructionHelper(src, dst, rows, cols, r+i, c+j);
                        }
                    }
                }
            }
        }
    }

    bool checkObjectShape(const Mat _src, int shape)
    {
        Mat src, img;
        _src.copyTo(src);

        // normalize and convert to grayscale
        cvtColor(src, src, CV_BGR2GRAY);
        equalizeHist(src, src);
        cvtColor(src, src, CV_GRAY2BGR);
        GaussianBlur(src, src, Size(3, 3), 3, 0, BORDER_DEFAULT);
        cvtColor(src, src, CV_BGR2GRAY);
        src.convertTo(src, CV_8U);

        vector<Mat> training_imgs;

        switch (shape) {
            case TRIANGLE:
                training_imgs = triangle_imgs;
                break;
            default:
                return false;
                break;
        }

        double size_threshold = 0.7;
        int matches_threshold = 2;
        int found_matches = 0;

        for (vector<Mat>::iterator it = training_imgs.begin(); it != training_imgs.end(); it++)
        {
            for (int minHessian = 300; minHessian <= 700; minHessian += 100)
            {
                img = *it;

                //-- Step 1: Detect the keypoints using SURF Detector
                SurfFeatureDetector detector( minHessian );

                vector<KeyPoint> keypoints_object, keypoints_scene;

                detector.detect( img, keypoints_object );
                detector.detect( src, keypoints_scene );

                //-- Step 2: Calculate descriptors (feature vectors)
                SurfDescriptorExtractor extractor;

                Mat descriptors_object, descriptors_scene;

                extractor.compute( img, keypoints_object, descriptors_object );
                extractor.compute( src, keypoints_scene, descriptors_scene );

                //-- Step 3: Matching descriptor vectors using FLANN matcher
                FlannBasedMatcher matcher;
                vector< DMatch > matches;
                matcher.match( descriptors_object, descriptors_scene, matches );

                double max_dist = 0; double min_dist = 100;

                //-- Quick calculation of max and min distances between keypoints
                for( int i = 0; i < descriptors_object.rows; i++ )
                { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
                }

                printf("-- Max dist : %f \n", max_dist );
                printf("-- Min dist : %f \n", min_dist );

                //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
                vector< DMatch > good_matches;

                for( int i = 0; i < descriptors_object.rows; i++ )
                { if( matches[i].distance < 3*min_dist )
                 { good_matches.push_back( matches[i]); }
                }

                Mat img_matches;
                drawMatches( img, keypoints_object, src, keypoints_scene,
                           good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                           vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                //-- Localize the object
                vector<Point2f> obj;
                vector<Point2f> scene;

                for( int i = 0; i < good_matches.size(); i++ )
                {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
                }

                if (obj.size() != 0 && scene.size() != 0)
                {
                    Mat H = findHomography( obj, scene, CV_RANSAC );

                    //-- Get the corners from the image_1 ( the object to be "detected" )
                    vector<Point2f> obj_corners(4);
                    obj_corners[0] = cvPoint(0,0);
                    obj_corners[1] = cvPoint( img.cols, 0 );
                    obj_corners[2] = cvPoint( img.cols, img.rows );
                    obj_corners[3] = cvPoint( 0, img.rows );
                    vector<Point2f> scene_corners(4);

                    perspectiveTransform( obj_corners, scene_corners, H);
                    Rect boundaries = boundingRect(Mat(scene_corners));

                    if (boundaries.width >= src.cols*size_threshold && boundaries.height >= src.rows*size_threshold)
                    {
                        found_matches++;
                    }
                }
            }
        }

        cout << found_matches << " matches for shape " << shape << endl;

        if (found_matches >= matches_threshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_object_detection_node");
    ImageObjectDetectionNode iodn;
    ros::spin();
    return 0;
}


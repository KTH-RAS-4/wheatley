#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const string OPENCV_WINDOW = "Object recognizer";
static const string DEBUG_WINDOW = "Image debug";

Mat mean_john = (Mat_<float>(1, 3) << 69.2250, 88.1010, 107.1206);
Mat sigma_john = (Mat_<float>(3, 3) << 2366.6, 2457.3, 2491.7, 2457.3, 2725.9, 2790.8, 2491.7, 2790.8, 2926.9);

Mat mean_cube_green = (Mat_<float>(1, 4) << 0.972656344995489, 0.20100263289924, 0.46319108796594, 141.562112717616);
Mat sigma_cube_green = (Mat_<float>(4, 4) << 0.000653839729695567, -0.00266341903225608, -0.00187619989376125, 0.749122343286518, -0.00266341903225608, 0.0128838271640458, 0.0116563083889695, -4.32140742183352, -0.00187619989376125, 0.0116563083889695, 0.035108942101794, -6.44534216882738, 0.749122343286518, -4.32140742183352, -6.44534216882738, 2710.56071965939);

Mat mean_wall = (Mat_<float>(1, 4) << 0.885019815723871, 0.278371710768153, 0.167802733922411, 203.088048503664);
Mat sigma_wall = (Mat_<float>(4, 4) << 0.0294730355218137, -0.0567022778756712, 0.016123850687772, 0.646092507340406, -0.0567022778756712, 0.111735886745609, -0.0317810954783104, -1.27912797018081, 0.016123850687772, -0.0317810954783104, 0.0189863480272751, 0.603175517400294, 0.646092507340406, -1.27912797018081, 0.603175517400294, 918.374047797582);

class ObjectRecognizerNode
{
    ros::NodeHandle handle;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    Mat frame_image;
    Mat templ;
    Mat result;
    int match_method;
    int cols;
    int rows;
    Size work_size;

public:
    ObjectRecognizerNode()
    : it(handle)
    {
        image_sub = it.subscribe("/camera/rgb/image_raw", 1, &ObjectRecognizerNode::imageHandle, this);

        templ = imread("../img/object_001.jpg", CV_LOAD_IMAGE_COLOR);
        cols = 0;
        rows = 0;
        work_size = Size(320, 240);

        namedWindow(OPENCV_WINDOW);
        //namedWindow(DEBUG_WINDOW);
    }

    ~ObjectRecognizerNode()
    {
        destroyWindow(OPENCV_WINDOW);
        //destroyWindow(DEBUG_WINDOW);
    }

    void imageHandle(const sensor_msgs::ImageConstPtr& msg)
    {
        static int i = 0;
        cv_bridge::CvImagePtr frame;

        try
        {
            frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            resize(frame->image, frame_image, work_size);

            char name[100];
            std::sprintf(name, "/tmp/images/%d.jpg", i++);
            cv::imwrite(name, frame->image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        /*result = gaussianLikelihood(frame_image, mean_green, sigma_green);
        double min, max;
        minMaxLoc(result, &min, &max);
        result.convertTo(result, CV_8U, 255.0/(max - min), -min * 255.0/(max - min));

        imshow(OPENCV_WINDOW, frame_image);
        imshow(DEBUG_WINDOW, result);
        */


        waitKey(2000);
    }

    Mat gaussianLikelihood(Mat &image_raw, Mat &mean, Mat &sigma)
    {
        Mat image = getAugmentedHSV(image_raw);
        //cvtColor(image_raw, image, CV_BGR2HSV);
        //image_raw.convertTo(image, CV_32FC3);
        Mat lvals;
        lvals.create(image.rows, image.cols, CV_32FC1);

        // the likelihood equation is divided to save time
        float object_a = 1.0/((pow(2.0*M_PI, 4.0/2.0))*sqrt(determinant(sigma)));
        float object_b;
        Mat sigma_inv = sigma.inv();

        float wall_a = 1.0/((pow(2.0*M_PI, 4.0/2.0))*sqrt(determinant(sigma_wall)));
        float wall_b;
        Mat sigma_wall_inv = sigma_wall.inv();

        float* _lvals;
        float* _image;
        Mat v;
        v.create(1, 4, CV_32FC1);
        
        for(int i = 0; i < lvals.rows; i++)
        {
            _lvals = lvals.ptr<float>(i);
            _image = image.ptr<float>(i);
            for(int j = 0; j < lvals.cols; j++)
            {
                // extract the pixel value
                v = (Mat_<float>(1, 4) << _image[j*4], _image[j*4+1], _image[j*4+2], _image[j*4+3]);

                object_b = exp(-0.5*(v-mean).t().dot(sigma_inv*(v-mean).t()));
                wall_b = exp(-0.5*(v-mean_wall).t().dot(sigma_wall_inv*(v-mean_wall).t()));
                //object_b = exp(-0.5*((Mat)((v-mean)*sigma_inv*(v-mean).t())).at<float>(0,0));

                if (object_a*object_b > wall_a*wall_b)
                {
                    _lvals[j] = 1;
                }
                else
                {
                    _lvals[j] = 0;
                }
            }
        }

        return lvals;
    }

    Mat getAugmentedHSV(Mat &image_raw)
    {
        // create augmented hsv matrix
        Mat hsv;
        hsv.create(image_raw.rows, image_raw.cols, CV_32FC4);
        
        Mat image;
        cvtColor(image_raw, image, CV_BGR2HSV);
        image.convertTo(image, CV_32FC3);

        float* _image;
        float* _hsv;
        for(int i = 0; i < image.rows; i++)
        {
            _image = image.ptr<float>(i);
            _hsv = hsv.ptr<float>(i);
            for(int j = 0; j < image.cols; j++)
            {
                _hsv[j*4] = cos(_image[j*3]);
                _hsv[j*4+1] = sin(_image[j*3]);
                _hsv[j*4+2] = _image[j*3+1];
                _hsv[j*4+3] = _image[j*3+2];
            }
        }

        return hsv;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_recognizer_node");
    ObjectRecognizerNode orn;
    ros::spin();
    return 0;
}

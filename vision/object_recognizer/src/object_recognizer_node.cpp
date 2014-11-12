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

Mat mean_green = (Mat_<float>(1, 3) << 36.8559, 102.5171, 88.9574);
Mat sigma_green = (Mat_<float>(3, 3) << 5568.6, 1986.7, 1853.1, 1986.7, 1360.7, 978.9, 1853.1, 978.9, 794.8);

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
        namedWindow(DEBUG_WINDOW);
    }

    ~ObjectRecognizerNode()
    {
        destroyWindow(OPENCV_WINDOW);
        destroyWindow(DEBUG_WINDOW);
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
        Mat image;
        image_raw.convertTo(image, CV_32FC3);
        Mat lvals;
        lvals.create(image.rows, image.cols, CV_32FC1);

        // the likelihood equation is divided to save time
        float term_a = 1.0/((pow(2.0*M_PI, 3.0/2.0))*sqrt(determinant(sigma)));
        float term_b;
        Mat sigma_inv = sigma.inv();

        float* _lvals;
        float* _image;
        Mat v;
        v.create(1, 3, CV_32FC1);
        
        for(int i = 0; i < lvals.rows; i++)
        {
            _lvals = lvals.ptr<float>(i);
            _image = image.ptr<float>(i);
            for(int j = 0; j < lvals.cols; j++)
            {
                // extract the pixel value
                v = (Mat_<float>(1, 3) << _image[j*3], _image[j*3+1], _image[j*3+2]);

                term_b = exp(-0.5*(v-mean).t().dot(sigma_inv*(v-mean).t()));
                _lvals[j] = term_a*term_b;
            }
        }

        return lvals;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_recognizer_node");
    ObjectRecognizerNode orn;
    ros::spin();
    return 0;
}

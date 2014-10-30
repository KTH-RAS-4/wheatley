#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Object recognizer";

class ObjectRecognizerNode
{
    ros::NodeHandle handle;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    cv::Mat templ;
    cv::Mat result;
    int match_method;
    int cols;
    int rows;

public:
    ObjectRecognizerNode()
    : it(handle)
    {
        image_sub = it.subscribe("/camera/rgb/image_raw", 1, &ObjectRecognizerNode::imageHandle, this);

        templ = cv::imread("../img/object_001.jpg", CV_LOAD_IMAGE_COLOR);
        cols = 0;
        rows = 0;

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ObjectRecognizerNode()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageHandle(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr frame;

        try
        {
            frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        if (cols == 0)
        {
            cols = frame->image.cols - templ.cols + 1;
            rows = frame->image.rows - templ.rows + 1;
            result.create(cols, rows, CV_32FC1);
        }

        //cv::matchTemplate(frame->image, templ, result, match_method);
        //cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

        cv::imshow(OPENCV_WINDOW, frame->image);

        cv::waitKey(3);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_recognizer_node");
    ObjectRecognizerNode orn;
    ros::spin();
    return 0;
}

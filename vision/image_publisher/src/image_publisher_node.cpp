#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle handle;
    image_transport::ImageTransport it(handle);
    image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);

    int device = 0;
    if (argc > 1)
    {
        device = std::atoi(argv[1]);
    }
    cv::VideoCapture cap(device);
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(10);
    while (handle.ok())
    {
        cap >> frame;
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
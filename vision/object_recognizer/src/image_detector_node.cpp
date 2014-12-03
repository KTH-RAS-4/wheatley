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

static const string WINDOW_1 = "Image mask";
static const string WINDOW_2 = "Image";

// objects
static const int WALL = 0x0;
static const int FLOOR = 0x1;
static const int RED = 0x2;
static const int ORANGE = 0x3;
static const int YELLOW = 0x4;
static const int GREEN = 0x5;
static const int BLUE = 0x6;
static const int PURPLE = 0x7;

// hue min max values
float object_color_hue[] = {
    0, 180, // wall
	0, 180, // floor
    -20, 0, // red
    0, 10, // orange
    17, 30, // yellow
    30, 80, // green
    80, 110, // blue
    110, 160 // purple
};

// saturation min max values
float object_color_saturation[] = {
    0, 120, // wall
	0, 255, // floor
    90, 255, // red
    120, 255, // orange
    127, 255, // yellow
    127, 255, // green
    120, 255, // blue
    80, 255 // purple
};

// value min max values
float object_color_value[] = {
    50, 255, // wall
	0, 255, // floor
    70, 255, // red
    180, 255, // orange
    127, 255, // yellow
    60, 255, // green
    50, 255, // blue
    80, 255 // purple
};

int mask_color_list[] = {
	WALL
};

int color_list[] = {
    ORANGE,
    YELLOW,
    PURPLE,
    RED,
    GREEN,
    BLUE
};

class ImageCorrectionNode
{
    ros::NodeHandle handle;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    Mat image;
    Mat Tmp;
    Mat Result;
    vector<Mat> channels;

    Mat erosion_element;
    Mat dilation_element;

public:
    ImageCorrectionNode()
    : it(handle)
    {
        image_sub = it.subscribe("/camera/rgb/image_raw", 1, &ImageCorrectionNode::imageHandle, this);

        namedWindow(WINDOW_1);
        namedWindow(WINDOW_2);

        int erosion_size = 6;
        erosion_element = getStructuringElement(MORPH_ELLIPSE, Size(2*erosion_size+1, 2*erosion_size+1), Point(erosion_size, erosion_size));
        int dilation_size = 5;
        dilation_element = getStructuringElement(MORPH_ELLIPSE, Size(2*dilation_size+1, 2*dilation_size+1), Point(dilation_size, dilation_size));
    }

    ~ImageCorrectionNode()
    {
        destroyWindow(WINDOW_1);
        destroyWindow(WINDOW_2);
    }

private:
    void imageHandle(const sensor_msgs::ImageConstPtr& msg)
    {
        static int i = 0;
        cv_bridge::CvImagePtr frame;

        try
        {
            frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            frame->image.copyTo(image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        // normalize the colors
        /*cvtColor(image, image, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format
        split(image, channels); //split the image into channels
        equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)
        merge(channels, image); //merge 3 channels including the modified 1st channel into one image
        cvtColor(image, image, CV_YCrCb2BGR);*/ //change the color image from YCrCb to BGR format (to display image properly)
        // apply gaussian filter
        GaussianBlur(image, image, Size(9, 9), 3, 0, BORDER_DEFAULT);

        // erosion
        //erode(image, image, erosion_element);
        // dilation
        //dilate(image, image, dilation_element);

        Mat mask;
		Mat masked;
        Mat marker;
		Mat edges;
        Result = Mat(image.rows, image.cols, CV_8UC3, Scalar(0,0,0));
        Vec3b bgr = Vec3b(0,0,0);
		Vec3b line_color = Vec3b(255,255,255);
        Vec3b* _Result;
		Vec3b* _image;
        float* _mask;
        unsigned char* _edges;
	
		// remove floor and walls
		image.copyTo(masked);
		for (int color = 0; color < (sizeof(mask_color_list)/sizeof(*mask_color_list)); color++)
        {
            mask = maskColor(image, mask_color_list[color]);

            for (int i = 0; i < image.rows; i++)
            {
                _image = masked.ptr<Vec3b>(i);
                _mask = mask.ptr<float>(i);
                for (int j = 0; j < image.cols; j++)
                {
                    if (_mask[j] > 0)
                    {
                        _image[j] = bgr;
                    }
                }
            }
        }

		//erode(image, image, erosion_element);

		// detect objects
        for (int color = 0; color < (sizeof(color_list)/sizeof(*color_list)); color++)
        {
            mask = maskColor(image, color_list[color]);
            erode(mask, marker, erosion_element);
            mask = maskReconstruction(marker, mask);

            switch(color_list[color])
            {
				case WALL: bgr = Vec3b(255,255,255); break;
				case FLOOR: bgr = Vec3b(0,70,130); break;
                case RED: bgr = Vec3b(0,0,255); break;
                case ORANGE: bgr = Vec3b(0,127,255); break;
                case YELLOW: bgr = Vec3b(0,255,255); break;
                case GREEN: bgr = Vec3b(0,255,0); break;
                case BLUE: bgr = Vec3b(255,0,0); break;
                case PURPLE: bgr = Vec3b(255,0,127); break;
            }

            for (int i = 0; i < image.rows; i++)
            {
                _Result = Result.ptr<Vec3b>(i);
                _mask = mask.ptr<float>(i);
                //_edges = edges.ptr<unsigned char>(i);
                for (int j = 0; j < image.cols; j++)
                {
                    if (_mask[j] > 0)
                    {
                        _Result[j] = bgr;
                    }
					/*if (_edges[j] > 0)
					{
						_Result[j] = line_color;
					}*/
                }
            }
        }

        //mask = maskColor(image, BLUE);
        
        //double min, max;
        //minMaxLoc(mask, &min, &max);
        //mask.convertTo(mask, CV_8U, 255.0/(max - min), -min * 255.0/(max - min));

		// edge detection
		Mat gray;
		cvtColor(masked, gray, CV_BGR2GRAY);
		Canny(gray, edges, 50, 100, 3, false);

		/// Find contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
		/// Draw contours
		Mat drawing = Mat::zeros(edges.size(), CV_8UC3);
/*		for( int i = 0; i< contours.size(); i++ )
		{
			drawContours(drawing, contours, i, Scalar(255,255,255), 2, 8, hierarchy, 0, Point());
		}*/
		// find polygons
		vector<vector<Point> > polygons;
		vector<Point> polygon;
		for (std::vector<vector<Point> >::iterator contour = contours.begin(); contour != contours.end(); contour++)
		{
			approxPolyDP(*contour, polygon, 2, true);
			polygons.push_back(polygon);
			cout << "vertices: " << polygon.size() << endl;
		}
		polylines(drawing, polygons, true, Scalar(255,255,255), 1, 8, 0);

        imshow(WINDOW_1, drawing);
        imshow(WINDOW_2, image);

        waitKey(10);
    }

    Mat maskColor(const Mat src, int color)
    {
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
                    maskReconstructionHelper(_mask, _mat, rows, cols, i, j);
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

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_correction_node");
    ImageCorrectionNode icn;
    ros::spin();
    return 0;
}


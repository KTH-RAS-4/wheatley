#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

static const string OPENCV_WINDOW = "Object recognizer";
static const string DEBUG_WINDOW = "Image debug";

Mat mean_john = (Mat_<float>(1, 4) << 0.993662546574559, 0.096830380934806, 0.407266033030462, 107.200697356742);
Mat sigma_john = (Mat_<float>(4, 4) << 0.000347348057998142, -0.000822653988603313, 0.000948326357033142, -0.239854600316153, -0.000822653988603313, 0.00291129012545325, -0.00566395160469576, 1.32716921244701, 0.000948326357033142, -0.00566395160469576, 0.0266213013520376, -5.67325445306638, -0.239854600316153, 1.32716921244701, -5.67325445306638, 2385.26876478163);

Mat mean_cube_green = (Mat_<float>(1, 4) << 0.972656344995489, 0.20100263289924, 0.46319108796594, 141.562112717616);
Mat sigma_cube_green = (Mat_<float>(4, 4) << 0.000653839729695567, -0.00266341903225608, -0.00187619989376125, 0.749122343286518, -0.00266341903225608, 0.0128838271640458, 0.0116563083889695, -4.32140742183352, -0.00187619989376125, 0.0116563083889695, 0.035108942101794, -6.44534216882738, 0.749122343286518, -4.32140742183352, -6.44534216882738, 2710.56071965939);

Mat mean_wall = (Mat_<float>(1, 4) << 0.885019815723871, 0.278371710768153, 0.167802733922411, 203.088048503664);
Mat sigma_wall = (Mat_<float>(4, 4) << 0.0294730355218137, -0.0567022778756712, 0.016123850687772, 0.646092507340406, -0.0567022778756712, 0.111735886745609, -0.0317810954783104, -1.27912797018081, 0.016123850687772, -0.0317810954783104, 0.0189863480272751, 0.603175517400294, 0.646092507340406, -1.27912797018081, 0.603175517400294, 918.374047797582);

Mat filter_mask = (Mat_<float>(3, 3) << 1, 2, 1, 2, 4, 2, 1, 2, 1);
float filter_weight = 16; // TODO: calculate this

Mat Gx = (Mat_<float>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
Mat Gy = (Mat_<float>(3, 3) << 1, 2, 1, 0, 0, 0, -1, -2, -1);

class ObjectRecognizerNode
{
    ros::NodeHandle handle;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    Mat frame_image;
    Mat Temp;
    Mat Result;
    int match_method;
    int cols;
    int rows;
    Size work_size;

    Scalar color_red;

public:
    ObjectRecognizerNode()
    : it(handle)
    {
        image_sub = it.subscribe("/camera/rgb/image_raw", 1, &ObjectRecognizerNode::imageHandle, this);

        work_size = Size(320, 240);

        color_red = Scalar(255, 0, 0);

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

            //char name[100];
            //std::sprintf(name, "/tmp/images/%d.jpg", i++);
            //cv::imwrite(name, frame->image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        // convert to gray scale
        cvtColor(frame_image, Temp, CV_BGR2GRAY);
        Temp.convertTo(Temp, CV_32FC1);

        // apply gaussian filter
        GaussianBlur(Temp, Temp, Size(5, 5), 1.4, 0, BORDER_DEFAULT);

        int cols = Temp.cols;
        int rows = Temp.rows;

        float edge_threshold = 50;

        //Result.copyTo(Temp);
        //Result = Mat(Temp.rows, Temp.cols, CV_32FC1);
        Mat Angles = Mat(rows, cols, CV_32FC1);
        Mat Strength = Mat(rows, cols, CV_32FC1);
        Mat Magnitude = Mat(rows, cols, CV_32FC1);
        float gx, gy, angle, edge_strength, edge_magnitude;
        float* _Gx = Gx.ptr<float>(0);
        float* _Gy = Gy.ptr<float>(0);
        float* _Temp;
        float* _Angles;
        float* _Strength;
        float* _Magnitude;
        for(int i = 0; i < rows; i++)
        {
            _Angles = Angles.ptr<float>(i);
            _Strength = Strength.ptr<float>(i);
            _Magnitude = Magnitude.ptr<float>(i);
            for(int j = 0; j < cols; j++)
            {
                gx = 0;
                gy = 0;
                for(int k = 0; k < 3; k++)
                {
                    if (i+k-1 >= 0 && i+k-1 < rows)
                    {
                        _Temp = Temp.ptr<float>(i+k-1);
                        for (int l = 0; l < 3; l++)
                        {
                            if (j+l-1 >= 0 && j+l-1 < cols)
                            {
                                gx += _Gx[k*3+l]*_Temp[j+l-1];
                                gy += _Gy[k*3+l]*_Temp[j+l-1];
                            }
                        }
                    }
                }
                edge_strength = abs(gx) + abs(gy);
                edge_magnitude = sqrt(gx*gx+gy*gy);
                // calculate the angle [0..pi]
                angle = 0;
                if (gx == 0) {
                    angle = M_PI/2.0;
                } else {
                    angle = atan(gy/gx);
                    if (angle < 0) {
                        angle += M_PI;
                    }
                }
                // quantify the angle
                if (angle >= M_PI*7.0/8.0 || angle <= M_PI*1.0/8.0)
                {
                    _Angles[j] = 1;
                }
                else if (angle > M_PI*5.0/8.0)
                {
                    _Angles[j] = 4;
                }
                else if (angle > M_PI*3.0/8.0)
                {
                    _Angles[j] = 3;
                }
                else if (angle > M_PI*1.0/8.0)
                {
                    _Angles[j] = 2;
                }

                _Strength[j] = edge_strength;
                _Magnitude[j] = edge_magnitude;
            }
        }

        Result = Mat::zeros(rows, cols, CV_32FC1);
        _Strength = Strength.ptr<float>(0);
        _Magnitude = Magnitude.ptr<float>(0);
        float* _Result = Result.ptr<float>(0);
        // find the edge key points
        for(int i = 1; i < rows-1; i++)
        {
            _Angles = Angles.ptr<float>(i);
            _Magnitude = Magnitude.ptr<float>(i);
            for(int j = 1; j < cols-1; j++)
            {
                if (_Magnitude[j] > edge_threshold)
                {
                    edge_strength = _Strength[i*cols+j];
                    if (_Angles[j] == 1)
                    {
                        // check top & bottom
                        if (edge_strength > _Strength[(i-1)*cols+j] && edge_strength > _Strength[(i+1)*cols+j])
                        {
                            _Result[(i-1)*cols+j] = 0;
                            _Result[(i+1)*cols+j] = 0;
                            _Result[i*cols+j] = 1;
                        }
                    }
                    else if (_Angles[j] == 2)
                    {
                        // check top left & bottom right
                        if (edge_strength > _Strength[(i-1)*cols+j-1] && edge_strength > _Strength[(i+1)*cols+j+1])
                        {
                            _Result[(i-1)*cols+j-1] = 0;
                            _Result[(i+1)*cols+j+1] = 0;
                            _Result[i*cols+j] = 1;
                        }
                    }
                    else if (_Angles[j] == 3)
                    {
                        // check left & right
                        if (edge_strength > _Strength[i*cols+j-1] && edge_strength > _Strength[i*cols+j+1])
                        {
                            _Result[i*cols+j-1] = 0;
                            _Result[i*cols+j+1] = 0;
                            _Result[i*cols+j] = 1;
                        }
                    }
                    else if (_Angles[j] == 4)
                    {
                        // check top right & bottom left
                        if (edge_strength > _Strength[(i-1)*cols+j+1] && edge_strength > _Strength[(i+1)*cols+j-1])
                        {
                            _Result[(i-1)*cols+j+1] = 0;
                            _Result[(i+1)*cols+j-1] = 0;
                            _Result[i*cols+j] = 1;
                        }
                    }
                }
            }
        }
        
        //Result.copyTo(Temp);
        _Result = Result.ptr<float>(0);
        _Magnitude = Magnitude.ptr<float>(0);
        _Angles = Angles.ptr<float>(0);
        // grow the edges from the key points
        for(int iterations=0; iterations<10; iterations++)
        {
        for(int i = 2; i < rows-2; i++)
        {
            for(int j = 2; j < cols-2; j++)
            {
                if (_Result[i*cols+j] == 1)
                {
                    angle = _Angles[i*cols+j];
                    if (angle == 1)
                    {
                        // check top
                        edge_strength = _Strength[(i-1)*cols+j];
                        if (angle == _Angles[(i-1)*cols+j] && _Magnitude[(i-1)*cols+j] > edge_threshold && edge_strength > _Strength[(i-1)*cols+j-1] && edge_strength > _Strength[(i-1)*cols+j+1])
                        {
                            _Result[(i-1)*cols+j-1] = 0;
                            _Result[(i-1)*cols+j+1] = 0;
                            _Result[(i-1)*cols+j] = 1;
                        }
                        // check bottom
                        edge_strength = _Strength[(i+1)*cols+j];
                        if (angle == _Angles[(i+1)*cols+j] && _Magnitude[(i+1)*cols+j] > edge_threshold && edge_strength > _Strength[(i+1)*cols+j-1] && edge_strength > _Strength[(i+1)*cols+j+1])
                        {
                            _Result[(i+1)*cols+j-1] = 0;
                            _Result[(i+1)*cols+j+1] = 0;
                            _Result[(i+1)*cols+j] = 1;
                        }
                    }
                    else if (angle == 2)
                    {
                        // check top left
                        edge_strength = _Strength[(i-1)*cols+j-1];
                        if (angle == _Angles[(i-1)*cols+j-1] && _Magnitude[(i-1)*cols+j-1] > edge_threshold && edge_strength > _Strength[i*cols+j-2] && edge_strength > _Strength[(i-2)*cols+j])
                        {
                            _Result[i*cols+j-2] = 0;
                            _Result[(i-2)*cols+j] = 0;
                            _Result[i*cols+j-1] = 0;
                            _Result[(i-1)*cols+j] = 0;
                            _Result[(i-1)*cols+j-1] = 1;
                        }
                        // check bottom right
                        edge_strength = _Strength[(i+1)*cols+j+1];
                        if (angle == _Angles[(i+1)*cols+j+1] && _Magnitude[(i+1)*cols+j+1] > edge_threshold && edge_strength > _Strength[(i+2)*cols+j] && edge_strength > _Strength[i*cols+j+2])
                        {
                            _Result[(i+2)*cols+j] = 0;
                            _Result[i*cols+j+2] = 0;
                            _Result[(i+1)*cols+j] = 0;
                            _Result[i*cols+j+1] = 0;
                            _Result[(i+1)*cols+j+1] = 1;
                        }
                    }
                    else if (angle == 3)
                    {
                        // check left
                        edge_strength = _Strength[i*cols+j-1];
                        if (angle == _Angles[i*cols+j-1] && _Magnitude[i*cols+j-1] > edge_threshold && edge_strength > _Strength[(i-1)*cols+j-1] && edge_strength > _Strength[(i+1)*cols+j-1])
                        {
                            _Result[(i-1)*cols+j-1] = 0;
                            _Result[(i+1)*cols+j-1] = 0;
                            _Result[i*cols+j-1] = 1;
                        }
                        // check right
                        edge_strength = _Strength[i*cols+j+1];
                        if (angle == _Angles[i*cols+j+1] && _Magnitude[i*cols+j+1] > edge_threshold && edge_strength > _Strength[(i-1)*cols+j+1] && edge_strength > _Strength[(i+1)*cols+j+1])
                        {
                            _Result[(i-1)*cols+j+1] = 0;
                            _Result[(i+1)*cols+j+1] = 0;
                            _Result[i*cols+j+1] = 1;
                        }
                    }
                    else if (angle == 4)
                    {
                        // check top right
                        edge_strength = _Strength[(i-1)*cols+j+1];
                        if (angle == _Angles[(i-1)*cols+j+1] && _Magnitude[(i-1)*cols+j+1] > edge_threshold && edge_strength > _Strength[(i-2)*cols+j] && edge_strength > _Strength[i*cols+j+2])
                        {
                            _Result[(i-2)*cols+j] = 0;
                            _Result[i*cols+j+2] = 0;
                            _Result[(i-1)*cols+j] = 0;
                            _Result[i*cols+j+1] = 0;
                            _Result[(i-1)*cols+j+1] = 1;
                        }
                        // check bottom left
                        edge_strength = _Strength[(i+1)*cols+j-1];
                        if (angle == _Angles[(i+1)*cols+j-1] && _Magnitude[(i+1)*cols+j-1] > edge_threshold && edge_strength > _Strength[i*cols+j-2] && edge_strength > _Strength[(i+2)*cols+j])
                        {
                            _Result[i*cols+j-2] = 0;
                            _Result[(i+2)*cols+j] = 0;
                            _Result[i*cols+j-1] = 0;
                            _Result[(i+1)*cols+j] = 0;
                            _Result[(i+1)*cols+j-1] = 1;
                        }
                    }
                }
            }
        }
        }



        double min, max;
        minMaxLoc(Result, &min, &max);
        Result.convertTo(Result, CV_8U, 255.0/(max - min), -min * 255.0/(max - min));

        imshow(OPENCV_WINDOW, Result);

        waitKey(10);
    }

    float getVectorAngle(float x, float y)
    {
        if (x != 0 || y != 0) {
            float angle = atan(-y/abs(x));
            if (x < 0) {
                if (angle < 0) {
                    angle = -angle;
                } else {
                    angle = M_PI-angle;
                }
            } else {
                if (angle < 0) {
                    angle += M_PI;
                }
            }
            return angle;
        }
        return 0;
    }

    void checkLightestDarkest(float val, float& lightest, float& darkest)
    {
        if (val > lightest)
        {
            lightest = val;
        }
        if (val < darkest)
        {
            darkest = val;
        }
    }

    bool isLocalMaximum32FC1(float* mat, int rows, int cols, int row, int col)
    {
        bool maximum = true;
        for (int i=-1; i<=1; i++)
        {
            for (int j=-1; j<=1; j++)
            {
                if (mat[row*cols+col] < mat[(row+i)*cols+col+j])
                {
                    maximum = false;
                    i = 2;
                    j = 2;
                }
            }
        }
        return maximum;
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

                object_b = exp(-0.5*((Mat)((v-mean)*sigma_inv*(v-mean).t())).at<float>(0,0));
                wall_b = exp(-0.5*((Mat)((v-mean_wall)*sigma_wall_inv*(v-mean_wall).t())).at<float>(0,0));
                //object_b = exp(-0.5*((Mat)((v-mean)*sigma_inv*(v-mean).t())).at<float>(0,0));

                _lvals[j] = object_a*object_b;
                /*if (object_a*object_b >= 0.1)
                {
                    _lvals[j] = 1;
                }
                else
                {
                    _lvals[j] = 0;
                }*/
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

        float deg2rad = M_PI/180;
        float* _image;
        float* _hsv;
        for(int i = 0; i < image.rows; i++)
        {
            _image = image.ptr<float>(i);
            _hsv = hsv.ptr<float>(i);
            for(int j = 0; j < image.cols; j++)
            {
                _hsv[j*4] = cos(_image[j*3]*deg2rad);
                _hsv[j*4+1] = sin(_image[j*3]*deg2rad);
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


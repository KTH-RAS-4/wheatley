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

// constants
float EDGE_THRESHOLD = 50; // edge threshold
float EIGEN_THRESHOLD = 70000; // corner threshold
int LINE_THRESHOLD = 6; // minimum number of points in a line

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
    Mat Angles;
    Mat Strength;
    Mat Magnitude;
    Mat Gradients;
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

private:
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
        GaussianBlur(Temp, Temp, Size(3, 3), 1.4, 0, BORDER_DEFAULT);

        int cols = Temp.cols;
        int rows = Temp.rows;

        //Result.copyTo(Temp);
        //Result = Mat(Temp.rows, Temp.cols, CV_32FC1);
        Angles = Mat(rows, cols, CV_32FC1);
        Strength = Mat(rows, cols, CV_32FC1);
        Magnitude = Mat(rows, cols, CV_32FC1);
        Gradients = Mat(rows, cols, CV_32FC2);
        float gx, gy, angle, edge_strength, edge_magnitude;
        float* _Gx = Gx.ptr<float>(0);
        float* _Gy = Gy.ptr<float>(0);
        float* _Temp;
        float* _Angles;
        float* _Strength;
        float* _Magnitude;
        float* _Gradients;
        for(int i = 0; i < rows; i++)
        {
            _Angles = Angles.ptr<float>(i);
            _Strength = Strength.ptr<float>(i);
            _Magnitude = Magnitude.ptr<float>(i);
            _Gradients = Gradients.ptr<float>(i);
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
                _Gradients[j*2] = gx;
                _Gradients[j*2+1] = gy;
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
                if (_Magnitude[j] > EDGE_THRESHOLD)
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
                        if (angle == _Angles[(i-1)*cols+j] && _Magnitude[(i-1)*cols+j] > EDGE_THRESHOLD && edge_strength > _Strength[(i-1)*cols+j-1] && edge_strength > _Strength[(i-1)*cols+j+1])
                        {
                            _Result[(i-1)*cols+j-1] = 0;
                            _Result[(i-1)*cols+j+1] = 0;
                            _Result[(i-1)*cols+j] = 1;
                        }
                        // check bottom
                        edge_strength = _Strength[(i+1)*cols+j];
                        if (angle == _Angles[(i+1)*cols+j] && _Magnitude[(i+1)*cols+j] > EDGE_THRESHOLD && edge_strength > _Strength[(i+1)*cols+j-1] && edge_strength > _Strength[(i+1)*cols+j+1])
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
                        if (angle == _Angles[(i-1)*cols+j-1] && _Magnitude[(i-1)*cols+j-1] > EDGE_THRESHOLD && edge_strength > _Strength[i*cols+j-2] && edge_strength > _Strength[(i-2)*cols+j])
                        {
                            _Result[i*cols+j-2] = 0;
                            _Result[(i-2)*cols+j] = 0;
                            _Result[i*cols+j-1] = 0;
                            _Result[(i-1)*cols+j] = 0;
                            _Result[(i-1)*cols+j-1] = 1;
                        }
                        // check bottom right
                        edge_strength = _Strength[(i+1)*cols+j+1];
                        if (angle == _Angles[(i+1)*cols+j+1] && _Magnitude[(i+1)*cols+j+1] > EDGE_THRESHOLD && edge_strength > _Strength[(i+2)*cols+j] && edge_strength > _Strength[i*cols+j+2])
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
                        if (angle == _Angles[i*cols+j-1] && _Magnitude[i*cols+j-1] > EDGE_THRESHOLD && edge_strength > _Strength[(i-1)*cols+j-1] && edge_strength > _Strength[(i+1)*cols+j-1])
                        {
                            _Result[(i-1)*cols+j-1] = 0;
                            _Result[(i+1)*cols+j-1] = 0;
                            _Result[i*cols+j-1] = 1;
                        }
                        // check right
                        edge_strength = _Strength[i*cols+j+1];
                        if (angle == _Angles[i*cols+j+1] && _Magnitude[i*cols+j+1] > EDGE_THRESHOLD && edge_strength > _Strength[(i-1)*cols+j+1] && edge_strength > _Strength[(i+1)*cols+j+1])
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
                        if (angle == _Angles[(i-1)*cols+j+1] && _Magnitude[(i-1)*cols+j+1] > EDGE_THRESHOLD && edge_strength > _Strength[(i-2)*cols+j] && edge_strength > _Strength[i*cols+j+2])
                        {
                            _Result[(i-2)*cols+j] = 0;
                            _Result[i*cols+j+2] = 0;
                            _Result[(i-1)*cols+j] = 0;
                            _Result[i*cols+j+1] = 0;
                            _Result[(i-1)*cols+j+1] = 1;
                        }
                        // check bottom left
                        edge_strength = _Strength[(i+1)*cols+j-1];
                        if (angle == _Angles[(i+1)*cols+j-1] && _Magnitude[(i+1)*cols+j-1] > EDGE_THRESHOLD && edge_strength > _Strength[i*cols+j-2] && edge_strength > _Strength[(i+2)*cols+j])
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

        // trace edge lines
        Result.copyTo(Temp);
        frame_image.convertTo(Result, CV_32FC3);
        vector<vector<int> > lines;
        int npoints;
        int pi, pj;
        _Angles = Angles.ptr<float>(0);
        _Temp = Temp.ptr<float>(0);
        _Result = Result.ptr<float>(0);
        for(int i = 1; i < rows-1; i++)
        {
            for(int j = 1; j < cols-1; j++)
            {
                if (_Temp[i*cols+j] == 1)
                {
                    vector<int> line = vector<int>();
                    line.push_back(i);
                    line.push_back(j);
                    pi = i;
                    pj = j;
                    npoints = 0;
                    while (_Temp[pi*cols+pj] == 1)
                    {
                        _Temp[pi*cols+pj] = 0;
                        for(int k = -1; k <= 1; k++)
                        {
                            for(int l = -1; l <= 1; l++)
                            {
                                if (_Temp[(pi+k)*cols+(pj+l)] == 1)
                                {
                                    line.push_back(pi+k);
                                    line.push_back(pj+l);
                                    npoints++;
                                    pi = pi+k;
                                    pj = pj+l;
                                    k = 2;
                                    l = 2;
                                }
                            }
                        }
                    }
                    // remove non lines
                    if (npoints >= LINE_THRESHOLD)
                    {
                        lines.push_back(line);
                        for (vector<int>::iterator lineit = line.begin(); lineit != line.end(); lineit+=2)
                        {
                            _Result[((*lineit)*cols+(*(lineit+1)))*3] = 0;
                            _Result[((*lineit)*cols+(*(lineit+1)))*3+1] = 255;
                            _Result[((*lineit)*cols+(*(lineit+1)))*3+2] = 0;
                        }
                    }
                }
            }
        }

        // corner detection
        Mat C = Mat(2, 2, CV_32FC1);
        Mat Eigen;
        float* _C = C.ptr<float>(0);
        float* _Eigen;
        float corners = 0;
        _Gradients = Gradients.ptr<float>(0);
        for(int i = 1; i < rows-1; i++)
        {
            _Result = Result.ptr<float>(i);
            for(int j = 1; j < cols-1; j++)
            {
                // fill with zeros
                _C[0] = _C[1] = _C[2] = _C[3] = 0;
                for(int k = -1; k <= 1; k++)
                {
                    for(int l = -1; l <= 1; l++)
                    {
                        gx = _Gradients[((i+k)*cols+j+l)*2];
                        gy = _Gradients[((i+k)*cols+j+l)*2+1];
                        _C[0] += gx*gx;
                        _C[1] += gx*gy;
                        _C[2] += gx*gy;
                        _C[3] += gy*gy;
                    }
                }
                // calculate eigen values
                eigen(C, Eigen, -1, -1);
                _Eigen = Eigen.ptr<float>(0);
                if (_Eigen[0] > EIGEN_THRESHOLD && _Eigen[1] > EIGEN_THRESHOLD)
                {
                    rectangle(Result, cvPoint(j-2,i-2), cvPoint(j+2,i+2), CV_RGB(255, 0, 0), 1, 8);
                    corners++;
                }
            }
        }

        cout << "lines: " << lines.size() << " corners: " << corners << endl;

        //double min, max;
        //minMaxLoc(Result, &min, &max);
        Result.convertTo(Result, CV_8UC3);//CV_8U, 255.0/(max - min), -min * 255.0/(max - min));

        imshow(OPENCV_WINDOW, Result);

        waitKey(10);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_recognizer_node");
    ObjectRecognizerNode orn;
    ros::spin();
    return 0;
}


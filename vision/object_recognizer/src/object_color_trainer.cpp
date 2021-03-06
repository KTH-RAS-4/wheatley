#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/types.h>
#include <dirent.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void printMatrix(Mat mat)
{
	for (int i = 0; i < mat.cols; i++)
	{
		for (int j = 0; j < mat.rows; j++)
		{
			cout << mat.at<float>(Point(i, j)) << " ";
		}
		cout << endl;
	}
}

int main(int argc, char** argv)
{
	/*
	unsigned char isFile = 0x8;
	char* path = argv[1];
	DIR* dir;
	struct dirent* entry;

	// count the files in the directory
	int file_count = 0;
	dir = opendir(path);
	entry = readdir(dir);
	while (entry)
	{
		if (entry->d_type == isFile)
		{
			cout << entry->d_name << endl;
			file_count++;
		}
		entry = readdir(dir);
	}

	Mat image;
	Mat samples(3, file_count, CV_32FC1);

	// read the image files
	dir = opendir(path);
	entry = readdir(dir);
	Vec<float, 3> color;
	int file_index = 0;
	char file_path[512];
	
	while (entry)
	{
		if (entry->d_type == isFile)
		{
			file_path[0] = '\0';
			strcat(file_path, path);
			strcat(file_path, "/");
			strcat(file_path, entry->d_name);
			cout << file_path << endl;
			image = imread(file_path, CV_LOAD_IMAGE_COLOR);

			color = Vec<float, 3>(0, 0, 0);
			for (int i = 0; i < image.cols; i++) {
				for (int j = 0; j < image.rows; j++) {
					color += Vec<float, 3>(image.at<Vec3b>(Point(i, j)));
				}
			}
			color /= image.cols * image.rows;
			//cout << color << endl;
			samples.at<float>(Point(file_index, 0)) = color[0];
			samples.at<float>(Point(file_index, 1)) = color[1];
			samples.at<float>(Point(file_index, 2)) = color[2];
			//samples.at<Vec3b>(0, file_index) = color[1];
			//samples.at<Vec3b>(2, file_index) = color[2];
			file_index++;
		}
		entry = readdir(dir);
	}
	printMatrix(samples);
	
	Mat covar;
	Mat mean;
	calcCovarMatrix(samples, covar, mean, CV_COVAR_SCRAMBLED | CV_COVAR_ROWS, CV_32FC1);

	cout << "color matrix" << endl;
	cout << "covar" << endl;
	printMatrix(covar);
	cout << endl << "mean" << endl;
	printMatrix(mean);
	*/

	cout << endl;
	Mat a = (Mat_<float>(3,1) << 1, 2, 3);
	Mat b = (Mat_<float>(1,3) << 1, 2, 3);
	Mat c = (Mat_<float>(2,2) << 2, 1, 1, 2);
	Mat d = (Mat_<float>(3, 3) << 1853.1, 1986.7, 5568.6, 978.9, 1360.7, 1986.7, 794.8, 978.9, 1853.1);

	Mat m(2, 2, CV_32FC4);
	Mat v;
	float* _m;
    for(int i = 0; i < m.rows; i++)
    {
        _m = m.ptr<float>(i);
        for(int j = 0; j < m.cols; j++)
        {
            _m[j*4] = 0;
            _m[j*4+1] = 1;
            v = (Mat_<float>(1, 4) << _m[j*4], _m[j*4+1], _m[j*4+2], _m[j*4+3]);
			cout << v << endl;
        }
    }

	return 0;
}
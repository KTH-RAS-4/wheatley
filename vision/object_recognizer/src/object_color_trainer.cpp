#include <iostream>
#include <string>
#include <vector>
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
			cout << mat.at<double>(Point(i, j)) << " ";
		}
		cout << endl;
	}
}

int main(int argc, char** argv)
{
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
	Mat samples(file_count, 3, CV_32FC1);

	// read the image files
	dir = opendir(path);
	entry = readdir(dir);
	Vec<double, 3> color;
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
			cout << path << endl;
			cout << file_path << endl;
			image = imread(file_path, CV_LOAD_IMAGE_COLOR);

			color = Vec<double, 3>(0, 0, 0);
			for (int i = 0; i < image.cols; i++) {
				for (int j = 0; j < image.rows; j++) {
					color += Vec<double, 3>(image.at<Vec3b>(Point(i, j)));
				}
			}
			color /= image.cols * image.rows;
			cout << color << endl;
			samples.at<double>(file_index, 0) = color[0];
			samples.at<double>(file_index, 1) = color[1];
			samples.at<double>(file_index, 2) = color[2];
			file_index++;
		}
		entry = readdir(dir);
	}
	
	Mat covar;
	Mat mean;
	calcCovarMatrix(&samples, file_count, covar, mean, 0, CV_64F);

	cout << "covar" << endl;
	printMatrix(covar);
	cout << endl << "mean" << endl;
	printMatrix(mean);

	return 0;
}
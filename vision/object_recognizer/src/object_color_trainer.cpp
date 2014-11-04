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
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			cout << mat.at<double>(i, j) << " ";
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
			file_count++;
		}
		entry = readdir(dir);
	}

	Mat mat;
	Mat samples(file_count, 3, CV_32FC1);

	// read the image files
	dir = opendir(path);
	entry = readdir(dir);

	while (entry)
	{
		if (entry->d_type == isFile)
		{
			// TODO: calculate the average pixel value
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
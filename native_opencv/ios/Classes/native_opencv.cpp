#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp> //Include OpenCV header file
#include <ctime>
#include <string>
#include <algorithm>

using namespace std;
using namespace cv;


void rotateMat(Mat &matImage, int rotation)
{
	if (rotation == 90)
	{
		transpose(matImage, matImage);
		flip(matImage, matImage, 1); // transpose+flip(1)=CW
	}
	else if (rotation == 270)
	{
		transpose(matImage, matImage);
		flip(matImage, matImage, 0); // transpose+flip(0)=CCW
	}
	else if (rotation == 180)
	{
		flip(matImage, matImage, -1); // flip(-1)=180
	}
}

extern "C"
{
	// Attributes to prevent 'unused' function from being removed and to make it visible
	__attribute__((visibility("default"))) __attribute__((used))
	const char *
	version()
	{
		return CV_VERSION;
	}
}
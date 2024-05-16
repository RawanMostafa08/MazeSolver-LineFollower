#ifndef car
#define car

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "utils.h"



bool robotFront(cv::Mat& image,cv::Point2f& front);
bool robotBack(cv::Mat& image,cv::Point2f& back);
bool detectcar(cv::Mat& image,cv::Mat& linedImage);

#endif
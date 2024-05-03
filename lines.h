#ifndef LINES
#define LINES

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


bool draw(cv::Mat  image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter);

bool draw_angle(cv::Mat  image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter);


#endif
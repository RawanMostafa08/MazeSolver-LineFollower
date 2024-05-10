#ifndef LINES
#define LINES

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


bool draw(cv::Mat  image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter);

bool draw_angle(cv::Mat  image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter);
cv::Mat thin_sheet(cv::Mat image);
cv::Mat GetLines(cv::Mat sheet);
bool Desicion(cv::Mat  image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter);
bool Desicion2(cv::Mat image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter);



#endif
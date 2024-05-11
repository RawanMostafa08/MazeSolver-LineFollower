#ifndef UTILS
#define UTILS

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


cv::Point2f computePerpendicularVector(const cv::Point2f &vec);
cv::Point2f computeDirectionVector(const cv::Point2f &pointA, const cv::Point2f &pointB);
int CalculateDistance(cv::Point p1, cv::Point p2);
cv::Point calculateMidpoint(cv::Point p1, cv::Point p2);
float calculateSlope(const cv::Point2f &p1, const cv::Point2f &p2);
cv::Mat edgesDetection(cv::Mat image);
cv::Mat extractImage(cv ::Mat image);


#endif
#ifndef UTILS
#define UTILS

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

bool inittrack(cv::Mat &image, cv::Mat &returnd);
cv::Point2f computePerpendicularVector(const cv::Point2f &vec);
cv::Point2f computeDirectionVector(const cv::Point2f &pointA, const cv::Point2f &pointB);
int CalculateDistance(cv::Point p1, cv::Point p2);
cv::Point calculateMidpoint(cv::Point p1, cv::Point p2);
float calculateSlope(const cv::Point2f &p1, const cv::Point2f &p2);\

cv::Mat extractImage(cv ::Mat& image);
bool isAngleWithinRange(double m1, double m2, double angleMinDeg, double angleMaxDeg);
cv::Mat thin_sheet(cv::Mat& image);
cv::Mat GetLines(cv::Mat&  sheet);
bool Desicion(cv::Mat& image, cv::Point2f&  frontrobotCenter, cv::Point2f&  backrobotCenter);
bool Desicion2(cv::Mat&  image, cv::Point2f&  frontrobotCenter, cv::Point2f&  backrobotCenter);
#endif
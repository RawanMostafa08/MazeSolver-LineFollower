#include "utils.h"

float calculateSlope(const cv::Point2f &p1, const cv::Point2f &p2)
{
    if (p2.x - p1.x == 0)
    {
        // Vertical line, slope is infinite
        return INFINITY;
    }
    return (p2.y - p1.y) / (p2.x - p1.x);
}
cv::Point calculateMidpoint(cv::Point p1, cv::Point p2)
{
    return cv::Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}
int CalculateDistance(cv::Point p1, cv::Point p2)
{
    return (p1.x - p2.x) ^ 2 + (p1.y - p2.y) ^ 2;
}
cv::Point2f computeDirectionVector(const cv::Point2f &pointA, const cv::Point2f &pointB)
{
    return (pointB - pointA);
}

// Function to compute a vector perpendicular to the given vector
cv::Point2f computePerpendicularVector(const cv::Point2f &vec)
{
    return cv::Point2f(-vec.y, vec.x);
}
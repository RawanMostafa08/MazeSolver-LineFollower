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
cv::Mat edgesDetection(cv::Mat image)
{
    // cvtColor(image, image, COLOR_HSV2BGR);
    cvtColor(image, image, cv::COLOR_BGR2GRAY); // Blur the image for better edge detection
    GaussianBlur(image, image, cv::Size(3, 3), 0);
    Canny(image, image, 100, 200, 3, false);
    std::vector<cv::Vec4i> lines;
    HoughLinesP(image, lines, 1, CV_PI / 20, 20, 10, 3);
    cv::Mat finalImage(image.size(), CV_8UC1, cv::Scalar(0, 0, 0));

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::line(finalImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    return finalImage;
}

cv::Mat extractImage(cv ::Mat image)
{

    cv::Mat gray;
    cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat thresholded;
    threshold(gray, thresholded, 160, 255, cv::THRESH_BINARY);

    cv::Mat edged_image;
    Canny(thresholded, edged_image, 150, 350);

    std::vector<std::vector<cv::Point>> contours;
    findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    int maxAreaContourIndex = -1;
    for (int i = 0; i < contours.size(); i++)
    {
        // std::cout << "contour" << std::endl;
        double area = contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxAreaContourIndex = i;
        }
    }
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    drawContours(mask, contours, maxAreaContourIndex, cv::Scalar(255), cv::FILLED);
    imshow("mask Sheet", mask);
    cv::waitKey(0);
    // Bitwise AND operation to extract white sheet
    cv::Mat extractedSheet;
    bitwise_and(image, image, extractedSheet, mask);
    return extractedSheet;
}
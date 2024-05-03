#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "car.h"
#include "lines.h"

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

int main(int, char **)
{

    std::string imagePath = "2.jpeg";
    cv::Mat image = cv::imread(imagePath);

    if (image.empty())
    {
        std::cerr << "Error: Unable to read the image." << std::endl;
        return -1;
    }
    cv::Size newSize(800, 800); // Resize the image
    cv::resize(image, image, newSize, cv::INTER_AREA);

    ////////////////////////////////////get edges///////////////////
    cv::Mat edgedImage = edgesDetection(image);
    ///////////////////////////////////////////////////////////////////
    // cvtColor(image, image, COLOR_BGR2HSV);
    // Mat img_blur;
    // GaussianBlur(image, img_blur, Size(3, 3), 0);
    ///////////////////////////////////////////////////////
    cv::Point2f frontrobotCenter = robotFront(image);
    cv::Point2f backrobotCenter = robotBack(image);
    std::cout << "the robot front" << frontrobotCenter << "the back" << backrobotCenter << std::endl;
    ////////////////////////////////////robot front and back detected/////////////////////
    if (draw(image, frontrobotCenter, backrobotCenter))
    {

        std::cout << "speed up" << std::endl;
    }
    else
    {
        std::cout << "slow down" << std::endl;
    }

    // cvtColor(image, image, COLOR_HSV2BGR);
    // Mat img_gray;
    // cvtColor(image, img_gray, COLOR_BGR2GRAY);
    // // Blur the image for better edge detection
    // GaussianBlur(img_gray, img_gray, Size(3, 3), 0);
    // Mat edges;
    // Canny(image, edges, 100, 200, 3, false);

    // std::vector<std::vector<Point>> contours;

    // findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // // Iterate over each pair of contours to draw mid-distance lines
    // std::vector<std::vector<Point>> potentialLines;
    // for (size_t i = 0; i < contours.size(); i++)
    // {
    //     double length = arcLength(contours[i], false);
    //     if (length > 100)
    //     {
    //         potentialLines.push_back(contours[i]);
    //     }
    // }
    // // Draw the potential lines on a blank image
    // Mat lineImage(edges.size(), CV_8UC1, Scalar(0, 0, 0));
    // for (int i = 0; i < potentialLines.size(); i++)
    // {
    //     drawContours(lineImage, potentialLines, i, Scalar(255, 255, 255), 2);
    // }

    // std::vector<Vec4i> lines;

    // HoughLinesP(lineImage, lines, 1, CV_PI / 180, 5, 1, 10);
    // Mat linedimage(edges.size(), CV_8UC1, Scalar(0, 0, 0));

    // // for (size_t i = 0; i < lines.size(); i++)
    // // {
    // //     Vec4i l = lines[i];
    // //     line(linedimage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(125, 125, 125), 2, LINE_AA);
    // // }
    // for (size_t i = 0; i < lines.size(); i++)
    // {
    //     for (size_t j = i + 1; j < lines.size(); j++)
    //     {
    //         Vec4i line1 = lines[i];
    //         Vec4i line2 = lines[j];
    //         double distanceXS = line1[0] - line2[0];
    //         double distanceYS = line1[1] - line2[1];
    //         double distanceXE = line1[3] - line2[3];
    //         // double distanceYE=line1[4]-line2[4];
    //         // std::cout<<distanceYE<<std::endl;
    //         if (abs(distanceXS) > 30 || abs(distanceYS) > 30 || abs(distanceXE) > 30)
    //         {
    //             continue;
    //         }
    //         // Check if the lines are approximately parallel
    //         double angle_diff = abs(atan2(line1[1] - line1[3], line1[0] - line1[2]) - atan2(line2[1] - line2[3], line2[0] - line2[2]));
    //         if (angle_diff < CV_PI / 5 || angle_diff > CV_PI - CV_PI / 5)
    //         { // Approximately parallel within 10 degrees
    //             Point midpoint1 = calculateMidpoint(Point(line1[0], line1[1]), Point(line2[0], line2[1]));
    //             Point midpoint2 = calculateMidpoint(Point(line1[2], line1[3]), Point(line2[2], line2[3]));
    //             line(linedimage, midpoint1, midpoint2, Scalar(255, 255, 255), 1); // Draw midpoint
    //         }
    //     }
    // }
    // imshow("original Image", linedimage);
    // waitKey(0);
}

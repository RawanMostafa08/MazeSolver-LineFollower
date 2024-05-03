#include "lines.h"
#include "utils.h"

bool draw(cv::Mat  image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter)
{

    int roiWidth = 500;
    int roiHeight = 500;

    // Define the region of interest (ROI) rectangle
    cv::Rect roiRect(frontrobotCenter.x - roiWidth / 2, frontrobotCenter.y - roiHeight / 2, roiWidth, roiHeight);

    // Ensure that the ROI rectangle is within the image boundaries
    roiRect &= cv::Rect(0, 0, image.cols, image.rows);

    // Extract the region of interest from the original image
    cv::Mat  roiImage = image(roiRect);
    // imshow("roiImage", roiImage);
    // cv::waitKey(0);

    // Perform edge detection on the ROI (you may already have this step)
    cv::Mat  edges;
    Canny(roiImage, edges, 50, 150, 3);
    imshow("edges", edges);
    cv::waitKey(0);
    // Detect lines using the Probabilistic Hough Transform
    std::vector<cv::Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 30, 10);
    cv::Mat  finalimage(edges.size(), CV_8UC1, cv::Scalar (0, 0, 0));
    float slope = calculateSlope(frontrobotCenter, backrobotCenter);
    bool speedup = false;
    // Draw the lines on the ROI image
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::Point p1 = cv::Point(l[0], l[1]);
        cv::Point p2 = cv::Point(l[2], l[3]);
        float diff = abs(slope - calculateSlope(p1, p2));
        if (diff < 1)
            if (CalculateDistance(frontrobotCenter, p1) > CalculateDistance(backrobotCenter, p1))
            {
                if (CalculateDistance(p2, p1) > 10)
                {
                    speedup = true;
                    line(finalimage, p1, p2, cv::Scalar (255, 255, 255), 2, cv::LINE_AA );
                }
            }
    }

    // Show the original image with the ROI and detected lines
    // rectangle(image, roiRect, cv::Scalar (0, 255, 0), 2); // Draw ROI rectangle on original image
    imshow("Original Image with ROI and Detected Lines", finalimage);
    cv::waitKey(0);
    return speedup;
}

bool draw_angle(cv::Mat  image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter)
{
    cv::Point2f directionVector = computeDirectionVector(backrobotCenter, frontrobotCenter);

    // Normalize the direction vector
    float magnitude = sqrt(directionVector.x * directionVector.x + directionVector.y * directionVector.y);
    directionVector.x /= magnitude;
    directionVector.y /= magnitude;

    // Scale the direction vector to set the dimensions of the ROI
    int roiWidth = 300;
    int roiHeight = 100;
    if (directionVector.x < directionVector.y)
    {
        cv::swap(roiHeight, roiWidth);
    }
    cv::Point2f topLeft = frontrobotCenter;
    if (frontrobotCenter.y == backrobotCenter.y && backrobotCenter.x > frontrobotCenter.x)
    {
        topLeft.x = frontrobotCenter.x - roiWidth;
        topLeft.y = frontrobotCenter.y - roiHeight / 2;
    }
    else if (frontrobotCenter.y == backrobotCenter.y && backrobotCenter.x < frontrobotCenter.x)
    {
        topLeft.y = frontrobotCenter.y - roiHeight / 2;
    }
    else if (frontrobotCenter.x == backrobotCenter.x && backrobotCenter.y > frontrobotCenter.y)
    {
        topLeft.x = frontrobotCenter.x - roiWidth / 2;
    }
    else if (frontrobotCenter.x == backrobotCenter.x && backrobotCenter.y < frontrobotCenter.y)
    {
        topLeft.x = frontrobotCenter.x - roiWidth / 2;
        topLeft.y = frontrobotCenter.y - roiHeight;
    }

    // Adjust the width and height based on the direction vector
    // cv::Point2f perpendicularVector = computePerpendicularVector(directionVector);
    // cv::Point2f roiCenter = (frontrobotCenter + backrobotCenter) * 0.5;
    // cv::Point2f topLeft = roiCenter - directionVector * roiWidth * 0.5 - perpendicularVector * roiHeight * 0.5;
    if (topLeft.x < 0)
        topLeft.x = 0;
    if (topLeft.y < 0)
        topLeft.y = 0;
    if (topLeft.x + roiWidth > image.cols)
        topLeft.x = image.cols - roiWidth;
    if (topLeft.y + roiHeight > image.rows)
        topLeft.y = image.rows - roiHeight;
    // // Define the region of interest (ROI) rectangle
    cv::Rect roiRect(topLeft.x, topLeft.y, roiWidth, roiHeight);
    std::cout << "ROI rectangle" << roiRect << std::endl;
    // // Extract the region of interest from the original image
    cv::Mat  roiImage = image(roiRect);

    // Perform edge detection on the ROI (you may already have this step)
    cv::Mat  edges;
    Canny(roiImage, edges, 50, 150, 3);
    imshow("Original Image with ROI and Detected Lines", edges);
    cv::waitKey(0);
    // Detect lines using the Probabilistic Hough Transform
    std::vector<cv::Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 30, 10);
    cv::Mat  finalimage(edges.size(), CV_8UC1, cv::Scalar (0, 0, 0));
    float slope = calculateSlope(frontrobotCenter, backrobotCenter);
    // Draw the lines on the ROI image
    bool speedup = false;
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::Point p1 = cv::Point(l[0], l[1]);
        cv::Point p2 = cv::Point(l[2], l[3]);
        float diff = abs(slope - calculateSlope(p1, p2));
        std::cout << diff << std::endl;
        if (diff < 1)
        {
            if (CalculateDistance(frontrobotCenter, p1) > CalculateDistance(backrobotCenter, p1))
            {

                if (CalculateDistance(p2, p1) > 10)
                {
                    speedup = true;
                    line(finalimage, p1, p2, cv::Scalar (255, 255, 255), 2, cv::LINE_AA );
                }
            }
        }
    }

    // Show the original image with the ROI and detected lines
    // rectangle(image, roiRect, cv::Scalar (0, 255, 0), 2); // Draw ROI rectangle on original image
    imshow("Original Image with ROI and Detected Lines", finalimage);
    cv::waitKey(0);
    return speedup;
}
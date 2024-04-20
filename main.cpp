#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
float calculateSlope(const Point2f &p1, const Point2f &p2)
{
    if (p2.x - p1.x == 0)
    {
        // Vertical line, slope is infinite
        return INFINITY;
    }
    return (p2.y - p1.y) / (p2.x - p1.x);
}
Point calculateMidpoint(Point p1, Point p2)
{
    return Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}
float CalculateDistance(Point p1, Point p2)
{
    return (p1.x - p2.x) ^ 2 + (p1.y - p2.y) ^ 2;
}
Point2f computeDirectionVector(const Point2f &pointA, const Point2f &pointB)
{
    return (pointB - pointA);
}

// Function to compute a vector perpendicular to the given vector
Point2f computePerpendicularVector(const Point2f &vec)
{
    return Point2f(-vec.y, vec.x);
}
Mat edgesDetection(Mat image)

{
    // cvtColor(image, image, COLOR_HSV2BGR);
    cvtColor(image, image, COLOR_BGR2GRAY); // Blur the image for better edge detection
    GaussianBlur(image, image, Size(3, 3), 0);
    Canny(image, image, 100, 200, 3, false);
    std::vector<Vec4i> lines;
    HoughLinesP(image, lines, 1, CV_PI / 20, 20, 10, 3);
    Mat finalImage(image.size(), CV_8UC1, Scalar(0, 0, 0));

    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(finalImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 2, LINE_AA);
    }
    return finalImage;
}
Point2f robotFront(Mat image)
{
    Mat fimg;

    cvtColor(image, fimg, COLOR_BGR2HSV);

    Scalar lower_red1 = Scalar(0, 50, 20);
    Scalar upper_red1 = Scalar(5, 255, 255);

    // Define the green color range that wraps around 0 degrees in HSV
    Scalar lower_red2 = Scalar(175, 50, 20);
    Scalar upper_red2 = Scalar(180, 255, 255);

    // Create masks for the two ranges
    Mat mask1, mask2;
    inRange(fimg, lower_red1, upper_red1, mask1);
    inRange(fimg, lower_red2, upper_red2, mask2);

    // Combine the masks using bitwise OR
    Mat mask;

    bitwise_or(mask1, mask2, mask);
    imshow("mask Image", mask);
    waitKey(0);

    // Find contours in the red mask
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat objectimage(fimg.size(), CV_8UC1, Scalar(0, 0, 0));

    float max_raduis = 30;
    Point2f max_center;
    for (int i = 0; i < contours.size(); i++)
    {
        Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        if (radius > 10 && radius < max_raduis)
        {
            max_raduis = radius;
            max_center = center;
        }
    }
    circle(objectimage, max_center, (int)max_raduis, Scalar(255, 255, 255), 2);

    imshow("object Image", objectimage);
    waitKey(0);
    return max_center;
}

Point2f robotBack(Mat image)
{
    Mat bimg;
    cvtColor(image, bimg, COLOR_BGR2HSV);
    // Define the green  color range in HSV
    Scalar lower_green1 = Scalar(35, 50, 20);
    Scalar upper_green1 = Scalar(85, 255, 255);

    // Define the green color range that wraps around 0 degrees in HSV
    Scalar lower_green2 = Scalar(145, 50, 20);
    Scalar upper_green2 = Scalar(180, 255, 255);

    // Create masks for the two ranges
    Mat mask1, mask2;
    inRange(bimg, lower_green1, upper_green1, mask1);
    inRange(bimg, lower_green2, upper_green2, mask2);

    // Combine the masks using bitwise OR
    Mat mask;

    bitwise_or(mask1, mask2, mask);
    imshow("mask Image", mask);
    waitKey(0);

    // Find contours in the red mask
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat objectimage(bimg.size(), CV_8UC1, Scalar(0, 0, 0));

    float max_raduis = 30;
    Point2f max_center;
    for (int i = 0; i < contours.size(); i++)
    {
        Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        if (radius > 10 && radius < max_raduis)
        {
            max_raduis = radius;
            max_center = center;
        }
    }
    circle(objectimage, max_center, (int)max_raduis, Scalar(255, 255, 255), 2);

    imshow("object Image", objectimage);
    waitKey(0);
    return max_center;
}

bool draw(Mat image, Point2f frontrobotCenter, Point2f backrobotCenter)
{

    int roiWidth = 500;
    int roiHeight = 500;

    // Define the region of interest (ROI) rectangle
    Rect roiRect(frontrobotCenter.x - roiWidth / 2, frontrobotCenter.y - roiHeight / 2, roiWidth, roiHeight);

    // Ensure that the ROI rectangle is within the image boundaries
    roiRect &= Rect(0, 0, image.cols, image.rows);

    // Extract the region of interest from the original image
    Mat roiImage = image(roiRect);
    // imshow("roiImage", roiImage);
    // waitKey(0);

    // Perform edge detection on the ROI (you may already have this step)
    Mat edges;
    Canny(roiImage, edges, 50, 150, 3);
    imshow("edges", edges);
    waitKey(0);
    // Detect lines using the Probabilistic Hough Transform
    std::vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 30, 10);
    Mat finalimage(edges.size(), CV_8UC1, Scalar(0, 0, 0));
    float slope = calculateSlope(frontrobotCenter, backrobotCenter);
    bool speedup = false;
    // Draw the lines on the ROI image
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        Point p1 = Point(l[0], l[1]);
        Point p2 = Point(l[2], l[3]);
        float diff = abs(slope - calculateSlope(p1, p2));
        if (diff < 1)
            if (CalculateDistance(frontrobotCenter, p1) > CalculateDistance(backrobotCenter, p1))
            {
                if (CalculateDistance(p2, p1) > 10)
                {
                    speedup = true;
                    line(finalimage, p1, p2, Scalar(255, 255, 255), 2, LINE_AA);
                }
            }
    }

    // Show the original image with the ROI and detected lines
    // rectangle(image, roiRect, Scalar(0, 255, 0), 2); // Draw ROI rectangle on original image
    imshow("Original Image with ROI and Detected Lines", finalimage);
    waitKey(0);
    return speedup;
}

bool draw_angle(Mat image, Point2f frontrobotCenter, Point2f backrobotCenter)
{
    Point2f directionVector = computeDirectionVector(backrobotCenter, frontrobotCenter);

    // Normalize the direction vector
    float magnitude = sqrt(directionVector.x * directionVector.x + directionVector.y * directionVector.y);
    directionVector.x /= magnitude;
    directionVector.y /= magnitude;

    // Scale the direction vector to set the dimensions of the ROI
    int roiWidth = 300;
    int roiHeight = 100;
    if (directionVector.x < directionVector.y)
    {
        swap(roiHeight, roiWidth);
    }
    Point2f topLeft = frontrobotCenter;
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
    // Point2f perpendicularVector = computePerpendicularVector(directionVector);
    // Point2f roiCenter = (frontrobotCenter + backrobotCenter) * 0.5;
    // Point2f topLeft = roiCenter - directionVector * roiWidth * 0.5 - perpendicularVector * roiHeight * 0.5;
    if (topLeft.x < 0)
        topLeft.x = 0;
    if (topLeft.y < 0)
        topLeft.y = 0;
    if (topLeft.x + roiWidth > image.cols)
        topLeft.x = image.cols - roiWidth;
    if (topLeft.y + roiHeight > image.rows)
        topLeft.y = image.rows - roiHeight;
    // // Define the region of interest (ROI) rectangle
    Rect roiRect(topLeft.x, topLeft.y, roiWidth, roiHeight);
    std::cout << "ROI rectangle" << roiRect << std::endl;
    // // Extract the region of interest from the original image
    Mat roiImage = image(roiRect);

    // Perform edge detection on the ROI (you may already have this step)
    Mat edges;
    Canny(roiImage, edges, 50, 150, 3);
    imshow("Original Image with ROI and Detected Lines", edges);
    waitKey(0);
    // Detect lines using the Probabilistic Hough Transform
    std::vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 30, 10);
    Mat finalimage(edges.size(), CV_8UC1, Scalar(0, 0, 0));
    float slope = calculateSlope(frontrobotCenter, backrobotCenter);
    // Draw the lines on the ROI image
    bool speedup = false;
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        Point p1 = Point(l[0], l[1]);
        Point p2 = Point(l[2], l[3]);
        float diff = abs(slope - calculateSlope(p1, p2));
        std::cout << diff << std::endl;
        if (diff < 1)
        {
            if (CalculateDistance(frontrobotCenter, p1) > CalculateDistance(backrobotCenter, p1))
            {

                if (CalculateDistance(p2, p1) > 10)
                {
                    speedup = true;
                    line(finalimage, p1, p2, Scalar(255, 255, 255), 2, LINE_AA);
                }
            }
        }
    }

    // Show the original image with the ROI and detected lines
    // rectangle(image, roiRect, Scalar(0, 255, 0), 2); // Draw ROI rectangle on original image
    imshow("Original Image with ROI and Detected Lines", finalimage);
    waitKey(0);
    return speedup;
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
    Mat edgedImage = edgesDetection(image);
    ///////////////////////////////////////////////////////////////////
    // cvtColor(image, image, COLOR_BGR2HSV);
    // Mat img_blur;
    // GaussianBlur(image, img_blur, Size(3, 3), 0);
    ///////////////////////////////////////////////////////
    Point2f frontrobotCenter = robotFront(image);
    Point2f backrobotCenter = robotBack(image);
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
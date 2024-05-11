#include "lines.h"
#include "utils.h"

cv::Mat thin_sheet(cv::Mat image)
{

    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    // Threshold the grayscale image to get a binary image
    cv::Mat binaryImage;
    cv::threshold(grayImage, binaryImage, 160, 255, cv::THRESH_BINARY);
    cv::bitwise_not(binaryImage, binaryImage); // Invert the binary image

    // Create a structuring element for morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    // Perform morphological erosion to thin the white lines
    cv::Mat thinnedImage;
    cv::erode(binaryImage, thinnedImage, kernel);

    // Perform skeletonization to further thin the lines
    cv::Mat skel(thinnedImage.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    bool done;
    do
    {
        cv::erode(thinnedImage, eroded, kernel);
        cv::dilate(eroded, temp, kernel);
        cv::subtract(thinnedImage, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(thinnedImage);

        done = (cv::countNonZero(thinnedImage) == 0);
    } while (!done);

    // Show the original and processed images
    cv::imshow("Processed Image", skel);
    cv::waitKey(0);
    return skel;
}
cv::Mat GetLines(cv::Mat sheet)
{

    std::vector<cv::Vec4i> lines;
    HoughLinesP(sheet, lines, 1, CV_PI / 180, 40, 18, 5);
    if (lines.empty())
    {
        return cv::Mat();
    }
    // HoughLinesP(image, lines, 1, CV_PI / 180, 50, 20, 5);
    cv::Mat LinedImage(sheet.size(), CV_8UC1, cv::Scalar(0, 0, 0));

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::line(LinedImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), 1, cv::LINE_AA);
    }

    cv::dilate(LinedImage, LinedImage, cv::Mat(), cv::Point(-1, -1), 2); // Adjust the kernel size as needed
    imshow("finalimage Sheet", LinedImage);
    cv::waitKey(0);

    return LinedImage;
}

bool Desicion(cv::Mat image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter)
{
    float roiWidth = 200;
    float roiHeight = 200;

    // Define the region of interest (ROI) rectangle
    cv::Rect roiRect(frontrobotCenter.x - roiWidth / 2, frontrobotCenter.y - roiHeight / 2, roiWidth, roiHeight);

    // Ensure that the ROI rectangle is within the image boundaries
    roiRect &= cv::Rect(0, 0, image.cols, image.rows);

    // Extract the region of interest from the original image
    cv::Mat roiImage = image(roiRect);
    imshow("roiImage", roiImage);
    cv::waitKey(0);
    // bool hasEdges = cv::countNonZero(roiImage) > 0;
    std::vector<cv::Vec4i> lines;

    HoughLinesP(roiImage, lines, 1, CV_PI / 180, 50, 30, 10);
    float slope = calculateSlope(frontrobotCenter, backrobotCenter);
    bool speedup = false;
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::Point p1 = cv::Point(l[0], l[1]);
        cv::Point p2 = cv::Point(l[2], l[3]);
        float diff = abs(slope - calculateSlope(p1, p2));
        if (diff < 1)
            if (CalculateDistance(frontrobotCenter, p1) > CalculateDistance(backrobotCenter, p1))
            {
                speedup = true;
            }
    }
    return speedup;
}
bool Desicion2(cv::Mat image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter)
{
    cv::Point2f directionVector = computeDirectionVector(backrobotCenter, frontrobotCenter);

    // Normalize the direction vector
    float magnitude = sqrt(directionVector.x * directionVector.x + directionVector.y * directionVector.y);
    directionVector.x /= magnitude;
    directionVector.y /= magnitude;

    // Scale the direction vector to set the dimensions of the ROI
    float roiWidth = 150;
    float roiHeight = 50;
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
    if (topLeft.x + roiWidth >= image.cols)
        topLeft.x = image.cols - roiWidth - 1;
    if (topLeft.y + roiHeight >= image.rows)
        topLeft.y = image.rows - roiHeight - 1;
    // // Define the region of interest (ROI) rectangle
    cv::Rect roiRect(topLeft.x, topLeft.y, roiWidth, roiHeight);
    std::cout << "ROI rectangle" << roiRect << std::endl;
    // Extract the region of interest from the original image
    cv::Mat roiImage = image(roiRect);
    imshow("roiImage", roiImage);
    cv::waitKey(0);
    bool speedup = cv::countNonZero(roiImage) > 0;

    return speedup;
}
bool draw(cv::Mat image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter)
{

    int roiWidth = 500;
    int roiHeight = 500;

    // Define the region of interest (ROI) rectangle
    cv::Rect roiRect(frontrobotCenter.x - roiWidth / 2, frontrobotCenter.y - roiHeight / 2, roiWidth, roiHeight);

    // Ensure that the ROI rectangle is within the image boundaries
    roiRect &= cv::Rect(0, 0, image.cols, image.rows);

    // Extract the region of interest from the original image
    cv::Mat roiImage = image(roiRect);
    // imshow("roiImage", roiImage);
    // cv::waitKey(0);

    // Perform edge detection on the ROI (you may already have this step)
    cv::Mat edges;
    Canny(roiImage, edges, 50, 150, 3);
    imshow("edges", edges);
    cv::waitKey(0);
    // Detect lines using the Probabilistic Hough Transform
    std::vector<cv::Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 30, 10);
    cv::Mat finalimage(edges.size(), CV_8UC1, cv::Scalar(0, 0, 0));
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
                    line(finalimage, p1, p2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                }
            }
    }

    // Show the original image with the ROI and detected lines
    // rectangle(image, roiRect, cv::Scalar (0, 255, 0), 2); // Draw ROI rectangle on original image
    imshow("Original Image with ROI and Detected Lines", finalimage);
    cv::waitKey(0);
    return speedup;
}

bool draw_angle(cv::Mat image, cv::Point2f frontrobotCenter, cv::Point2f backrobotCenter)
{
    cv::Point2f directionVector = computeDirectionVector(backrobotCenter, frontrobotCenter);

    // Normalize the direction vector
    float magnitude = sqrt(directionVector.x * directionVector.x + directionVector.y * directionVector.y);
    directionVector.x /= magnitude;
    directionVector.y /= magnitude;

    // Scale the direction vector to set the dimensions of the ROI
    float roiWidth = 300;
    float roiHeight = 100;
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
    cv::Mat roiImage = image(roiRect);

    // Perform edge detection on the ROI (you may already have this step)
    cv::Mat edges;
    Canny(roiImage, edges, 50, 150, 3);
    imshow("Original Image with ROI and Detected Lines", edges);
    cv::waitKey(0);
    // Detect lines using the Probabilistic Hough Transform
    std::vector<cv::Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 30, 10);
    cv::Mat finalimage(edges.size(), CV_8UC1, cv::Scalar(0, 0, 0));
    float slope = calculateSlope(frontrobotCenter, backrobotCenter);
    // Draw the lines on the ROI image
    bool speedup = false;
    for (int i = 0; i < lines.size(); i++)
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
                    line(finalimage, p1, p2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
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
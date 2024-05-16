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

cv::Mat extractImage(cv ::Mat &image)
{
    double area = static_cast<double>(image.rows) * image.cols;
    cv::Size newSize(800, 1000); // Resize the image
    cv::resize(image, image, newSize, cv::INTER_AREA);

    cv::Mat gray;
    cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat thresholded;
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    threshold(gray, thresholded, 160, 255, cv::THRESH_BINARY);
    imshow("thresholded", thresholded);
    cv::waitKey(0);
    cv::Mat edged_image;
    Canny(thresholded, edged_image, 150, 350);

    std::vector<std::vector<cv::Point>> contours;
    findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() < 1)
    {
        return cv::Mat();
    }
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
    cv::Mat contourMask = cv::Mat::zeros(thresholded.size(), CV_8UC1);
    cv::drawContours(contourMask, contours, maxAreaContourIndex, cv::Scalar(255), cv::FILLED);
    imshow("contourMask", contourMask);
    cv::waitKey(0);
    // Find bounding box of the largest contour
    cv::Rect boundingBox = cv::boundingRect(contours[maxAreaContourIndex]);

    // Crop the original image using the bounding box
    cv::Mat croppedImage = image(boundingBox).clone();
    // imshow("croppedImage", croppedImage);
    // cv::waitKey(0);
    // Apply mask to the cropped image to keep only the desired contour
    cv::Mat result;
    croppedImage.copyTo(result, contourMask(boundingBox));
    // imshow("result", result);
    // cv::waitKey(0);
    return croppedImage;
}

cv::Mat thin_sheet(cv::Mat &image)
{

    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayImage, grayImage, cv::Size(7, 7), 0);
    // Threshold the grayscale image to get a binary image
    cv::Mat binaryImage;
    cv::threshold(grayImage, binaryImage, 150, 255, cv::THRESH_BINARY);
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
    // cv::imshow("Processed Image", skel);
    // cv::waitKey(0);
    return skel;
}
cv::Mat GetLines(cv::Mat &sheet)
{

    cv::Canny(sheet, sheet, 50, 140, 3);

    std::vector<cv::Vec4i> lines;
    HoughLinesP(sheet, lines, 1, CV_PI / 180, 50, 20, 15);
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

    // cv::dilate(LinedImage, LinedImage, cv::Mat(), cv::Point(-1, -1), 1);
    // imshow("finalimage Sheet", LinedImage);
    // cv::waitKey(0);

    return LinedImage;
}
bool isAngleWithinRange(double m1, double m2, double angleMinDeg, double angleMaxDeg)
{
    const double PI = 3.14159265358979323846;

    // Convert angle range from degrees to radians
    double angleMinRad = angleMinDeg * (PI / 180.0);
    double angleMaxRad = angleMaxDeg * (PI / 180.0);

    // Calculate the angle between the two lines using the slopes
    double angle = std::atan(std::abs((m1 - m2) / (1 + m1 * m2)));

    // Check if the calculated angle is within the specified range
    return (angle >= angleMinRad && angle <= angleMaxRad);
}
bool Desicion(cv::Mat &image, cv::Point2f &frontrobotCenter, cv::Point2f &backrobotCenter)
{
    float roiWidth = 150;
    float roiHeight = 150;

    // Define the region of interest (ROI) rectangle
    cv::Rect roiRect(static_cast<int>(frontrobotCenter.x - roiWidth / 2),
                     static_cast<int>(frontrobotCenter.y - roiHeight / 2),
                     static_cast<int>(roiWidth),
                     static_cast<int>(roiHeight));

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
        float slp = calculateSlope(p1, p2);
        if (isAngleWithinRange(slope, slp, 0, 20))
            speedup = true;
    }
    // bool speedup = cv::countNonZero(roiImage) > 0;

    return speedup;
}
bool Desicion2(cv::Mat &image, cv::Point2f &frontrobotCenter, cv::Point2f &backrobotCenter)
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
    cv::Rect roiRect(static_cast<int>(topLeft.x),
                     static_cast<int>(topLeft.y),
                     static_cast<int>(roiWidth),
                     static_cast<int>(roiHeight));
    std::cout << "ROI rectangle" << roiRect << std::endl;
    // Extract the region of interest from the original image
    cv::Mat roiImage = image(roiRect);
    // imshow("roiImage", roiImage);
    // cv::waitKey(0);
    bool speedup = cv::countNonZero(roiImage) > 0;

    return speedup;
}

bool inittrack(cv::Mat &image, cv::Mat &returnd)
{
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    cv::Mat sheet = extractImage(image);
    if (sheet.empty())
    {
        std::cerr << "Error: Unable to extract the paper." << std::endl;
        return 0;
    }
    imshow("extracted", sheet);
    cv::waitKey(0);

    cv::Mat thinnedImage = thin_sheet(sheet);
    imshow("thinned", thinnedImage);
    cv::waitKey(0);
    returnd = GetLines(thinnedImage);
    if (returnd.empty())
    {
        std::cerr << "Error: Unable to extract the lines." << std::endl;
        return 0;
    }

    return 1;
}
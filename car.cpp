#include "car.h"

cv::Point2f robotFront(cv::Mat image)
{
    cv::Mat fimg;
    cv::GaussianBlur(image, fimg, cv::Size(5, 5), 0);

    cvtColor(fimg, fimg, cv::COLOR_BGR2HSV);

    cv::Scalar lower_red1 = cv::Scalar(0, 50, 20);
    cv::Scalar upper_red1 = cv::Scalar(5, 255, 255);

    // Define the green color range that wraps around 0 degrees in HSV
    cv::Scalar lower_red2 = cv::Scalar(175, 50, 20);
    cv::Scalar upper_red2 = cv::Scalar(180, 255, 255);

    // Create masks for the two ranges
    cv::Mat mask1, mask2;
    inRange(fimg, lower_red1, upper_red1, mask1);
    inRange(fimg, lower_red2, upper_red2, mask2);

    // Combine the masks using bitwise OR
    cv::Mat mask;

    bitwise_or(mask1, mask2, mask);
    imshow("mask Image", mask);
    cv::waitKey(0);

    // Find contours in the red mask
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
 if (contours.size() < 1)
    {
        return cv::Point2f(-1, -1);
    }
    cv::Mat objectimage(fimg.size(), CV_8UC1, cv::Scalar(0, 0, 0));

    float max_raduis = 30;
    cv::Point2f max_center;
    for (int i = 0; i < contours.size(); i++)
    {
        cv::Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        if (radius < max_raduis)
        {
            max_raduis = radius;
            max_center = center;
        }
    }
    circle(objectimage, max_center, (int)max_raduis, cv::Scalar(255, 255, 255), 2);

    imshow("object Image", objectimage);
    cv::waitKey(0);
    return max_center;
}

cv::Point2f robotBack(cv::Mat image)
{
    cv::Mat bimg;
    cv::GaussianBlur(image, bimg, cv::Size(5, 5), 0);
    cvtColor(bimg, bimg, cv::COLOR_BGR2HSV);
    // Define the green  color range in HSV
    cv::Scalar lower_green1 = cv::Scalar(35, 50, 20);
    cv::Scalar upper_green1 = cv::Scalar(85, 255, 255);

    // Define the green color range that wraps around 0 degrees in HSV
    cv::Scalar lower_green2 = cv::Scalar(40, 50, 20);
    cv::Scalar upper_green2 = cv::Scalar(80, 255, 255);

    // Create masks for the two ranges
    cv::Mat mask1, mask2;
    inRange(bimg, lower_green1, upper_green1, mask1);
    inRange(bimg, lower_green2, upper_green2, mask2);
    // Combine the masks using bitwise OR
    cv::Mat mask;

    bitwise_or(mask1, mask2, mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);  // Opening to remove noise
    morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel); // Closing to fill gaps

    imshow("mask Image", mask);
    cv::waitKey(0);

    // Find contours in the red mask
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() < 1)
    {
        return cv::Point2f(-1, -1);
    }
    cv::Mat objectimage(bimg.size(), CV_8UC1, cv::Scalar(0, 0, 0));

    float max_raduis = 30;
    cv::Point2f max_center;
    for (int i = 0; i < contours.size(); i++)
    {
        cv::Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        if (radius < max_raduis)
        {
            max_raduis = radius;
            max_center = center;
        }
    }
    circle(objectimage, max_center, (int)max_raduis, cv::Scalar(255, 255, 255), 2);

    imshow("object Image", objectimage);
    cv::waitKey(0);
    return max_center;
}
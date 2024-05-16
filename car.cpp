#include "car.h"

bool robotFront(cv::Mat &image, cv::Point2f &front)
{
    cv::Mat fimg;

    cv::GaussianBlur(image, fimg, cv::Size(5, 5), 0);
    // imshow("image Image", image);
    // cv::waitKey(0);

    cvtColor(fimg, fimg, cv::COLOR_BGR2HSV);
    //   imshow("fimg Image", fimg);
    //     cv::waitKey(0);
    cv::Scalar lower_red1 = cv::Scalar(0, 100, 100);
    cv::Scalar upper_red1 = cv::Scalar(10, 255, 255);

    // Define the green color range that wraps around 0 degrees in HSV
    cv::Scalar lower_red2 = cv::Scalar(160, 100, 100);
    cv::Scalar upper_red2 = cv::Scalar(180, 255, 255);

    // Create masks for the two ranges
    cv::Mat mask1, mask2;
    inRange(fimg, lower_red1, upper_red1, mask1);
    inRange(fimg, lower_red2, upper_red2, mask2);

    // Combine the masks using bitwise OR
    cv::Mat mask;

    bitwise_or(mask1, mask2, mask);
    // imshow("mask Image", mask);
    // cv::waitKey(0);

    // Find contours in the red mask
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() < 1)
    {
        return 0;
    }
    cv::Mat objectimage(fimg.size(), CV_8UC1, cv::Scalar(0, 0, 0));

    float max_raduis = 300;
    double max_area = 0;
    cv::Point2f max_center;
    for (int i = 0; i < contours.size(); i++)
    {
        cv::Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        double area = contourArea(contours[i]);

        if (area > max_area && radius < max_raduis)
        {
            max_raduis = radius;
            max_area = area;
            max_center = center;
        }
    }
    circle(objectimage, max_center, (int)max_raduis, cv::Scalar(255, 255, 255), 2);

    imshow("object Image", objectimage);
    cv::waitKey(0);
    front = max_center;
    return 1;
}

bool robotBack(cv::Mat &image, cv::Point2f &back)
{
    cv::Mat bimg;
    cv::GaussianBlur(image, bimg, cv::Size(5, 5), 0);
    cvtColor(bimg, bimg, cv::COLOR_BGR2HSV);
    // Define the green  color range in HSV
    cv::Scalar lower_yellow = cv::Scalar(20, 100, 100);
    cv::Scalar upper_yellow = cv::Scalar(30, 255, 255);

    // Define the lower and upper bounds for orange in HSV color space
    cv::Scalar lower_orange = cv::Scalar(25, 100, 100);
    cv::Scalar upper_orange = cv::Scalar(35, 255, 255);

    // Create masks for the two ranges
    cv::Mat mask1, mask2;
    inRange(bimg, lower_yellow, upper_yellow, mask1);
    inRange(bimg, lower_orange, upper_orange, mask2);
    // Combine the masks using bitwise OR
    cv::Mat mask;

    bitwise_or(mask1, mask2, mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);  // Opening to remove noise
    morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel); // Closing to fill gaps

    // imshow("mask Image", mask);
    // cv::waitKey(0);

    // Find contours in the red mask
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() < 1)
    {
        return 0;
    }
    cv::Mat objectimage(bimg.size(), CV_8UC1, cv::Scalar(0, 0, 0));
    double max_area = 0;
    float max_raduis = 30;
    cv::Point2f max_center;
    for (int i = 0; i < contours.size(); i++)
    {
        cv::Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        double area = contourArea(contours[i]);
        if (area > max_area)
        {
            max_area = area;
            max_center = center;
            max_raduis = radius;
        }
    }
    circle(objectimage, max_center, (int)max_raduis, cv::Scalar(255, 255, 255), 2);

    imshow("object Image", objectimage);
    cv::waitKey(0);
    back = max_center;
    return 1;
}
bool detectcar(cv::Mat &image, cv::Mat &linedImage)
{
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    cv::Mat sheet = extractImage(image);
    if (sheet.empty())
    {
        // std::cerr << "Error: Unable to extract the paper." << std::endl;
        return 0;
    }

    cv::cvtColor(sheet, sheet, cv::COLOR_BGR2RGB);
    cv::Point2f frontrobotCenter;
    if (!robotFront(sheet, frontrobotCenter))
    {
        return 0;
    }
    cv::Point2f backrobotCenter;
    if (!robotBack(sheet, backrobotCenter))
    {
        return 0;
    }
    if (Desicion(linedImage, frontrobotCenter, backrobotCenter))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

struct KittiData {
    cv::Mat leftImage;
    cv::Mat rightImage;
    double fx;          // x focal length in pixels
    double fy;          // y focal length in pixels
    double cx;          // x of principal point in pixels
    double cy;          // y of principal point in pixels
    double b;           // baseline in meters
};

KittiData getData(const std::string& imageName);
std::ostream& operator<<(std::ostream& os, const KittiData& s);

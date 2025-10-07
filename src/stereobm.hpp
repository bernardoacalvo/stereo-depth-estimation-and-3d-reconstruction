#pragma once
#include <opencv2/opencv.hpp>

cv::Mat computeStereobmDisparity(const cv::Mat& left, const cv::Mat& right, int numDisparities, int blockSize);
cv::Mat leftToRightDisparityCheck(const cv::Mat& leftDisparity, const cv::Mat& rightDisparity, const int THRESHOLD = 1);
cv::Mat computeDepth(const cv::Mat& disparity, const float focalLength, const float baseline);

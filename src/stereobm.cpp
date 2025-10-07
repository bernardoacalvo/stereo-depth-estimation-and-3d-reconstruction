#include <limits>
#include <opencv2/opencv.hpp>

#include "stereobm.hpp"

cv::Mat computeStereobmDisparity(const cv::Mat& left, const cv::Mat& right, int numDisparities, int blockSize) {
    // greyscale because StereoBM compares pixel intensity values only (if needed)
    cv::Mat leftGrey, rightGrey;
    if(left.channels() > 1)
        cv::cvtColor(left, leftGrey, cv::COLOR_BGR2GRAY);
    else
        leftGrey = left.clone();

    if(right.channels() > 1)
        cv::cvtColor(right, rightGrey, cv::COLOR_BGR2GRAY);
    else
        rightGrey = right.clone();

    cv::normalize(leftGrey, leftGrey, 0, 255, cv::NORM_MINMAX);
    cv::normalize(rightGrey, rightGrey, 0, 255, cv::NORM_MINMAX);

    const int rows = leftGrey.rows, cols = leftGrey.cols;
    const int half = blockSize / 2;
    cv::Mat disparity = cv::Mat::zeros(rows, cols, CV_8U);      // unsigned 8 bit integer = [0, 255] if grey scale
    
    // start and end where a full block fits, (x,y) are the center pixel, so they must start at blockSize/2 to have space for a full block
    // y = row index, x = column index
    // x = half + numDisparities to only consider patches until our max disparity range.
    for(int y = half; y < rows - half; y++) {
        for(int x = half + numDisparities; x < cols - half; x++) {
            int minSad = std::numeric_limits<int>::max();  // max infinity
            int bestD = 0;
            for(int d = 0; d < numDisparities; d++) {       // check for each possible disparity in a scanline (basically moving one pixel to the left for each patch)
                int sad = 0;
                if(x - d - half < 0)        // check if right image block is within image
                    continue;
                // create blocks
                for(int v = -half; v <= half; ++v) {
                    for(int u = -half; u <= half; ++u) {
                        int leftVal = leftGrey.at<uchar>(y+v, x+u);
                        int rightVal = rightGrey.at<uchar>(y+v, x+u-d);    // offset by disparity (right image "shifts" left)
                        sad += abs(leftVal - rightVal);       // sum of absolute differences
                    }
                }
                if(sad < minSad) {     // min sad = max similarity
                    minSad = sad;
                    bestD = d;
                }
            }
            disparity.at<uchar>(y,x) = static_cast<uchar>(bestD);
        }
    }
    // median filter to smooth disparity (kernel size is usually 3)
    cv::medianBlur(disparity, disparity, 3);
    return disparity;
}

cv::Mat leftToRightDisparityCheck(const cv::Mat& leftDisparity, const cv::Mat& rightDisparity, const int THRESHOLD) {
    // THRESHOLD = allowed disparity difference
    cv::Mat disparity = leftDisparity.clone();
    int rows = leftDisparity.rows;
    int cols = leftDisparity.cols;
    for(int y = 0; y < rows; y++) {
        for(int x = 0; x < cols; x++) {
            int dLeft = leftDisparity.at<uchar>(y, x);
            int xRight = x - dLeft;
            if(xRight >= 0 && xRight < cols) {        // inbound
                int dRight = rightDisparity.at<uchar>(y, xRight);
                if(abs(dLeft - dRight) > THRESHOLD)
                    disparity.at<uchar>(y, x) = 0;
            } else {
                disparity.at<uchar>(y, x) = 0;              // out of bounds - invalid
            }
        }
    }
    return disparity;
}

cv::Mat computeDepth(const cv::Mat& disparity, const float focalLength, const float baseline) {       // depth = (f * B) / d
    cv::Mat disparityFloat;
    disparity.convertTo(disparityFloat, CV_32F);
    // new disparity for safe division
    cv::Mat safeDisparity = disparityFloat.clone();
    safeDisparity.setTo(1.0f, safeDisparity <= 0.0f);
    // compute depth
    cv::Mat depth = (focalLength * baseline) / safeDisparity;
    // set invalid disparities to zero
    depth.setTo(0.0f, disparityFloat <= 0.0f);
    return depth;
}
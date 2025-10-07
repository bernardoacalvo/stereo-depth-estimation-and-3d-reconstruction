#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include "stereobm.hpp"
#include "pointcloud.hpp"
#include "kitti.hpp"

int main() {
    std::string RESULTS_PATH = "./results/";
    const std::string IMAGE_NAME = "000000_11";

    const int MAX_DISPARITIES = 64;
    const int BLOCK_SIZE = 9;

    KittiData data = getData(IMAGE_NAME);

    cv::Mat leftRefDisparity = computeStereobmDisparity(data.leftImage, data.rightImage, MAX_DISPARITIES, BLOCK_SIZE);

    //cv::Mat rightRefDisparity = computeStereobmDisparity(data.rightImage, data.leftImage, MAX_DISPARITIES, BLOCK_SIZE);
    //cv::Mat disparity = leftToRightDisparityCheck(leftRefDisparity, rightRefDisparity, 3);

    cv::Mat depth = computeDepth(leftRefDisparity, data.fx, data.b);

    cv::Mat disparityNorm, depthNorm;
    cv::normalize(leftRefDisparity, disparityNorm, 0, 255, cv::NORM_MINMAX, CV_8U);
    /*
    cv::imshow("Disparity Map", disparityNorm);
    cv::waitKey(0);
    */

    cv::normalize(depth, depthNorm, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat depthColor;
    cv::applyColorMap(depthNorm, depthColor, cv::COLORMAP_JET);
    /*
    cv::imshow("Depth Map", depthColor);
    cv::waitKey(0);
    */

    cv::imwrite(RESULTS_PATH+"image.png", data.leftImage);
    cv::imwrite(RESULTS_PATH+"disparity.png", disparityNorm);
    cv::imwrite(RESULTS_PATH+"depth.png", depthColor);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = pointCloudReconstruction(depth, data.leftImage, data.fx, data.fy, data.cx, data.cy);
    visualizePointCloud(pointCloud, RESULTS_PATH+"cloud.png");
}

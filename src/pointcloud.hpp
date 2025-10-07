#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudReconstruction(const cv::Mat& depth, const cv::Mat& image, float fx, float fy, float cx, float cy);
void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& saveImagePath = "cloud.png");
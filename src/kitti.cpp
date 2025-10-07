#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "kitti.hpp"

const std::string KITTI_PATH = "kitti/data_scene_flow/training/";
const std::string KITTI_CALIBRATION_PATH = "kitti/data_scene_flow_calib/training/calib_cam_to_cam/";

std::ostream& operator<<(std::ostream& os, const KittiData& data) {
    os << "KittiData(fx=" << data.fx << ", fy=" << data.fy << ", cx=" << data.cy << ", cy=" << data.cx << ", b=" << data.b << ")";
    return os;
}

std::vector<double> parseMatrixFromLine(const std::string& line) {
    std::istringstream iss(line);
    std::string tag;
    iss >> tag;         // skip tag (e.g. P_rect_02:)
    std::vector<double> vals;
    double v;
    while(iss >> v)
        vals.push_back(v);
    return vals;
}

KittiData getData(const std::string& imageName) {
    cv::Mat lImage = cv::imread(KITTI_PATH+"image_2/"+imageName+".png", cv::IMREAD_COLOR);
    cv::Mat rImage = cv::imread(KITTI_PATH+"image_3/"+imageName+".png", cv::IMREAD_COLOR);

    std::string calibImageName = imageName.substr(0, imageName.find('_'));
    std::ifstream infile(KITTI_CALIBRATION_PATH+calibImageName+".txt");
    if(!infile.is_open())
        throw std::runtime_error("Failed to open calibration file.");

    std::string line, p2Line, p3Line;
    while(std::getline(infile, line)) {
        if(line.find("P_rect_02:") == 0) 
            p2Line = line;
        else if(line.find("P_rect_03:") == 0)
            p3Line = line;
    }
    if(p2Line.empty() || p3Line.empty())
        throw std::runtime_error("Projection lines not found.");

    std::vector<double> P2 = parseMatrixFromLine(p2Line);
    std::vector<double> P3 = parseMatrixFromLine(p3Line);

    if (P2.size() != 12 || P3.size() != 12)
        throw std::runtime_error("Projection matrices are not of size 12.");

    double fx = P2[0];      // P2 = left image
    double fy = P2[5];
    double cx = P2[2];
    double cy = P2[6];

    double Tx2 = P2[3];
    double Tx3 = P3[3];
    double baseline = std::abs(Tx2 - Tx3) / fx;         // difference between cameras positions, in meters

    return {lImage, rImage, fx, fy, cx, cy, baseline};
}


/*
Matrix:
fx 0  cx Tx
0  fy cy Ty
0  0  1  Tz 
*/

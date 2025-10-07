#include <opencv2/opencv.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudReconstruction(
    const cv::Mat& depth, 
    const cv::Mat& image, 
    float fx, float fy, float cx, float cy) 
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int v = 0; v < depth.rows; v++) {
        for(int u = 0; u < depth.cols; u++) {
            float z = depth.at<float>(v, u);
            if(z <= 0.0f || std::isnan(z))      // skip invalid ones
                continue;
            
            pcl::PointXYZRGB point;
            point.x = (u - cx) * z / fx;        // similar triangles
            point.y = (v - cy) * z / fy;
            point.z = z;

            cv::Vec3b color = image.at<cv::Vec3b>(v, u);    // BGR
            point.r = color[2];
            point.g = color[1];
            point.b = color[0];

            cloud->points.push_back(point);     // add point
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& saveImagePath) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.4f, 0.4f, 0.4f);   // voxel size in meters
    sor.filter(*cloudFiltered);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudFiltered, "Point Cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point Cloud");
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(
        0, 0, -0.05,            // camera position (x, y, z)
        0, 0, 1,            // view direction vector (towards +Z)
        0, -1, 0            // up vector (image y axis points down (OpenCV), so up is -Y)
    );

    viewer->spinOnce(100); // render at least one frame

    viewer->saveScreenshot(saveImagePath);

    while (!viewer->wasStopped())
        viewer->spinOnce(100);
}
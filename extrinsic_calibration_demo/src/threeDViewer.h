#ifndef THREEDVIEWER_H_
#define THREEDVIEWER_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include "calibrationHandler.h"

class ThreeDViewer
{
public:
    ThreeDViewer(std::shared_ptr<CalibrationHandler> handler);
    ~ThreeDViewer();

    void SetUpPointClouds(std::shared_ptr<CalibrationHandler> &handler, const int &counter);
    void DrawCameraFrustum(pcl::visualization::PCLVisualizer::Ptr &viewer);
    void DrawSphereCenterIn2DImage(std::shared_ptr<CalibrationHandler> &handler, const int &cam_index, const int &frameCounter);
    void DrawSphereIn3DSpace(pcl::visualization::PCLVisualizer::Ptr &viewer, const int &cam_index, const int &frameCounter, std::string &sphereName);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;

    const std::vector<double> cam_color_0{1.0, 0.0, 0.0};
    const std::vector<double> cam_color_1{0.0, 1.0, 0.0};
    const std::vector<double> cam_color_2{0.0, 0.0, 1.0};

private:
    int _rows{0};
    int _cols{0};
    float _scale{0.001f};
    cv::Mat _inverseColorProjection;
    cv::Mat _colorProjection;
    std::vector<cv::Mat> _rotation;
    std::vector<cv::Mat> _translation;
    std::vector<cv::Mat> _cam_pose_rots;
    std::vector<cv::Mat> _cam_pose_trans;
    std::vector<cv::Mat> _path_mat_sphere;
};

#endif
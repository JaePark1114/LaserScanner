#pragma once
#ifndef POINTCLOUD_PLY_
#define POINTCLOUD_PLY_

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"


class Pointcloud
{
public:
    void clear(void);
    void init_points(int rows, int cols);
    void init_color(int rows, int cols);
    void init_normals(int rows, int cols);
    bool write_ply(const std::string& filename, Pointcloud& pointcloud, unsigned flags);
    void compute_normals(Pointcloud& pointcloud);
    static inline bool INVALID12(const cv::Vec2f& pt) { return _isnan(pt[0]) || _isnan(pt[1]); }
    static inline bool INVALID1(const cv::Vec3f& pt) { return _isnan(pt[0]) || _isnan(pt[1]) || _isnan(pt[2]); }

    //data
    cv::Mat points;
    cv::Mat colors;
    cv::Mat normals;
};

#endif // !


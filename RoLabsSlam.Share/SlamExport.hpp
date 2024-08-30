#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "types_c.h"
#include "my_functions.h"

// Forward declare Slam class to avoid including the full definition here
class Slam;

CVAPI(Slam*) Slam_create();
CVAPI(void) Slam_destroy(Slam* slam);

// Wrap Slam methods
CVAPI(void) Slam_grabImage(Slam* slam, cv::Mat* image);
CVAPI(void) Slam_getDebugKeyPoints(Slam* slam, std::vector<cv::KeyPoint>* keypointsCurrent, std::vector<cv::KeyPoint>* keypointsPrevious);
CVAPI(void) Slam_stop(Slam* slam);
CVAPI(void) Slam_start(Slam* slam);
CVAPI(void) Slam_setIntrinsicsMatrix(Slam* slam, float fx, float fy, float cx, float cy);
CVAPI(void) Slam_getCurrentPose(Slam* slam, cv::Mat* pose);
CVAPI(void) Slam_track(Slam* slam);
CVAPI(void) Slam_getMapPoints(Slam* slam, std::vector<cv::Point3d>* mapPoint);
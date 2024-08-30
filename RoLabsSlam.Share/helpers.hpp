#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <chrono>  // Include chrono for timing
#include "Frame.hpp"

extern void FindRtAndTriangulate(
    const cv::Mat& essential_matrix,
    const cv::Mat& cameraMatrix,
    const std::vector<cv::DMatch>& good_matches,
    std::shared_ptr<Frame> currentFrame,
    std::shared_ptr<Frame> previousFrame
    );


extern cv::Mat CreateTransformationMatrix(const cv::Mat& R, const cv::Mat& t);

extern std::vector<cv::DMatch> MatchKeyPoints(const Frame& frame1, const Frame& frame2);

extern cv::Mat EstimateEssentialMatrix(const Frame& frame1, const Frame& frame2, const std::vector<cv::DMatch>& good_matches, const cv::Mat& cameraMatrix);

extern std::vector<cv::Point3d> MyTriangulatePoints(
    const cv::Mat& P1, const cv::Mat& P2,
    const std::vector<cv::Point2f>& points1,
    const std::vector<cv::Point2f>& points2);

extern int SearchByProjection(std::shared_ptr<Frame> currentFrame, const std::set<std::shared_ptr<MapPoint>>& mapPoints, const cv::Mat& intrinsicCameraMatrix, int searchRadius);